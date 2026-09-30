#include <QApplication>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/tool.hpp>
#include <iostream>
int main(int argc, char **argv) {
  QApplication app(argc, argv);
  pluginlib::ClassLoader<rviz_common::Tool> loader("rviz_common", "rviz_common::Tool");
  for (const auto &name : {"legbot_rviz_tools/NavGoal3D", "legbot_rviz_tools/PoseEstimate3D"}) {
    auto tool = loader.createSharedInstance(name);
    if (!tool || !tool->getPropertyContainer()) return 1;
    std::cout << "Loaded " << name << std::endl;
  }
}
