// Read-only inspection of serialized Karto constraints; never a policy input.
#include <karto_sdk/Karto.h>
#include <karto_sdk/Mapper.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: export_slam_graph_edges INPUT.data INPUT.posegraph OUTPUT.csv\n";
    return 2;
  }
  try {
    karto::Dataset dataset;
    dataset.LoadFromFile(argv[1]);
    for (auto *object : dataset.GetLasers()) {
      auto *sensor = dynamic_cast<karto::Sensor *>(object);
      if (sensor) karto::SensorManager::GetInstance()->RegisterSensor(sensor);
    }
    karto::Mapper mapper;
    mapper.LoadFromFile(argv[2]);
    std::ofstream output(argv[3]);
    if (!output) throw std::runtime_error("Cannot create output CSV");
    output << "source_id,target_id,source_stamp,target_stamp,dx,dy,dyaw,"
              "cov_xx,cov_xy,cov_yy,cov_yawyaw,source_odom_x,source_odom_y,"
              "source_odom_yaw,target_odom_x,target_odom_y,target_odom_yaw\n";
    output << std::setprecision(17);
    size_t count = 0;
    for (auto *edge : mapper.GetGraph()->GetEdges()) {
      auto *source = edge->GetSource()->GetObject();
      auto *target = edge->GetTarget()->GetObject();
      auto *link = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
      if (!source || !target || !link) throw std::runtime_error("Invalid graph edge");
      const auto &delta = link->GetPoseDifference();
      const auto &cov = link->GetCovariance();
      const auto &a = source->GetOdometricPose();
      const auto &b = target->GetOdometricPose();
      output << source->GetUniqueId() << ',' << target->GetUniqueId() << ','
             << source->GetTime() << ',' << target->GetTime() << ','
             << delta.GetX() << ',' << delta.GetY() << ',' << delta.GetHeading() << ','
             << cov(0,0) << ',' << cov(0,1) << ',' << cov(1,1) << ',' << cov(2,2) << ','
             << a.GetX() << ',' << a.GetY() << ',' << a.GetHeading() << ','
             << b.GetX() << ',' << b.GetY() << ',' << b.GetHeading() << '\n';
      ++count;
    }
    if (!count) throw std::runtime_error("No graph edges");
    output.close();
    std::cerr << "Exported " << count << " finalized graph edges\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
