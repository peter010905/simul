// Offline scan-mapper component replay, using only recorded odometry and ranges.
// Finalized SLAM data are a retained scan subset, not the full original ROS stream.
#include <karto_sdk/Karto.h>
#include <karto_sdk/Mapper.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include <cstdlib>

struct Listener : karto::MapperLoopClosureListener {
  std::ofstream stream;
  double stamp = 0.;
  explicit Listener(const std::string &path) : stream(path) {}
  void Info(const std::string &value) override { stream << stamp << " INFO " << value << '\n'; }
  void LoopClosureCheck(const std::string &value) override { stream << stamp << " CHECK " << value << '\n'; }
  void BeginLoopClosure(const std::string &value) override { stream << stamp << " BEGIN " << value << '\n'; }
  void EndLoopClosure(const std::string &value) override { stream << stamp << " END " << value << '\n'; }
};

int main(int argc, char **argv) {
  if (argc != 6) {
    std::cerr << "Usage: replay_karto_component INPUT.data INPUT.posegraph OUTPUT_PREFIX UNTIL_SIM_S VARIANT\n";
    return 2;
  }
  try {
    const std::string prefix(argv[3]), variant(argv[5]);
    if (variant != "baseline" && variant != "no_loop" && variant != "strict_loop" && variant != "fine55" && variant != "huber")
      throw std::runtime_error("Unknown controlled variant");
    const double until = std::stod(argv[4]);
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("offline_karto_replay");
    node->declare_parameter("debug_logging", false);
    node->declare_parameter("ceres_linear_solver", "SPARSE_NORMAL_CHOLESKY");
    node->declare_parameter("ceres_preconditioner", "SCHUR_JACOBI");
    node->declare_parameter("ceres_dogleg_type", "TRADITIONAL_DOGLEG");
    node->declare_parameter("ceres_trust_strategy", "LEVENBERG_MARQUARDT");
    node->declare_parameter("ceres_loss_function", variant == "huber" ? "HuberLoss" : "None");
    node->declare_parameter("mode", "mapping");
    if (const char *threads = std::getenv("OSP_REPLAY_SOLVER_THREADS"))
      node->declare_parameter("osp_solver_threads", std::stoi(threads));
    pluginlib::ClassLoader<karto::ScanSolver> loader("slam_toolbox", "karto::ScanSolver");
    const char *selected_solver = std::getenv("OSP_REPLAY_SOLVER");
    auto solver = loader.createSharedInstance(selected_solver ? selected_solver : "solver_plugins::CeresSolver");
    solver->Configure(node);
    karto::Dataset dataset;
    dataset.LoadFromFile(argv[1]);
    for (auto *object : dataset.GetLasers()) {
      auto *sensor = dynamic_cast<karto::Sensor *>(object);
      if (sensor) karto::SensorManager::GetInstance()->RegisterSensor(sensor);
    }
    karto::Mapper original;
    original.LoadFromFile(argv[2]);
    // Own cloned scans until after mapper destruction. Never change originals.
    std::vector<std::unique_ptr<karto::LocalizedRangeScan>> scans;
    Listener listener(prefix+".events.log");
    karto::Mapper mapper;
    for (auto *parameter : original.GetParameters()) {
      auto *destination = mapper.GetParameter(parameter->GetName());
      if (!destination) throw std::runtime_error("Recorded parameter unavailable in mapper");
      destination->SetValueFromString(parameter->GetValueAsString());
    }
    if (variant == "no_loop") mapper.SetParameter("DoLoopClosing", false);
    if (variant == "fine55") mapper.SetParameter("LoopMatchMinimumResponseFine", .55);
    if (variant == "strict_loop") {
      mapper.SetParameter("LoopMatchMinimumResponseCoarse", .55);
      mapper.SetParameter("LoopMatchMinimumResponseFine", .65);
      mapper.SetParameter("LoopMatchMaximumVarianceCoarse", 1.);
    }
    std::ofstream parameters(prefix+".parameters.txt");
    for (auto *parameter : mapper.GetParameters()) parameters << *parameter << '\n';
    mapper.SetScanSolver(solver.get());
    mapper.AddListener(&listener);
    std::ofstream online(prefix+".online.csv");
    if (!online) throw std::runtime_error("Cannot create replay output");
    online << "original_node_id,node_id,stamp_sim_s,accepted,odom_x,odom_y,odom_yaw,map_x,map_y,map_yaw,edges\n";
    online << std::setprecision(17);
    size_t count = 0, accepted = 0;
    for (const auto &entry : dataset.GetData()) {
      auto *source = dynamic_cast<karto::LocalizedRangeScan *>(entry.second);
      if (!source || source->GetTime() > until) continue;
      auto scan = std::make_unique<karto::LocalizedRangeScan>(source->GetSensorName(), source->GetRangeReadingsVector());
      scan->SetTime(source->GetTime());
      scan->SetOdometricPose(source->GetOdometricPose());
      scan->SetCorrectedPose(source->GetOdometricPose());
      listener.stamp = source->GetTime();
      const bool ok = mapper.Process(scan.get());
      const auto &odom = scan->GetOdometricPose();
      const auto &pose = scan->GetCorrectedPose();
      online << source->GetUniqueId() << ',' << scan->GetUniqueId() << ',' << scan->GetTime() << ',' << ok << ','
             << odom.GetX() << ',' << odom.GetY() << ',' << odom.GetHeading() << ','
             << pose.GetX() << ',' << pose.GetY() << ',' << pose.GetHeading() << ','
             << mapper.GetGraph()->GetEdges().size() << '\n';
      scans.push_back(std::move(scan));
      ++count; accepted += ok;
      if (count%50 == 0) { online.flush(); listener.stream.flush(); }
    }
    mapper.RemoveListener(&listener);
    online.close();
    std::cerr << "Replayed " << count << " retained scans; accepted " << accepted << '\n';
    rclcpp::shutdown();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
