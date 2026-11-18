// Read-only export from SLAM Toolbox's serialized Karto dataset.
// No simulator truth, ROS services, or changes to the original graph.
#include <karto_sdk/Karto.h>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: export_slam_graph_poses INPUT.data OUTPUT.csv\n";
    return 2;
  }
  try {
    karto::Dataset dataset;
    dataset.LoadFromFile(argv[1]);
    // Deserialization restores pointers but does not register sensor names.
    // Dataset::Clear unregisters them, so mirror the loader's sensor setup.
    for (auto *object : dataset.GetLasers()) {
      auto *sensor = dynamic_cast<karto::Sensor *>(object);
      if (sensor) karto::SensorManager::GetInstance()->RegisterSensor(sensor);
    }
    std::ofstream output(argv[2]);
    if (!output) throw std::runtime_error("Cannot create output CSV");
    output << "node_id,stamp_sim_s,odom_x,odom_y,odom_yaw,map_x,map_y,map_yaw\n";
    output << std::setprecision(17);
    size_t count = 0;
    for (const auto &entry : dataset.GetData()) {
      const auto *scan = dynamic_cast<const karto::LocalizedRangeScan *>(entry.second);
      if (!scan) continue;
      const auto &odom = scan->GetOdometricPose();
      const auto &pose = scan->GetCorrectedPose();
      output << scan->GetUniqueId() << ',' << scan->GetTime() << ','
             << odom.GetX() << ',' << odom.GetY() << ',' << odom.GetHeading() << ','
             << pose.GetX() << ',' << pose.GetY() << ',' << pose.GetHeading() << '\n';
      ++count;
    }
    if (!count) throw std::runtime_error("Dataset contains no localized range scans");
    output.close();
    std::cerr << "Exported " << count << " finalized graph scan poses\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
