#include <cmath>
#include <memory>
#include <QKeyEvent>
#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_rendering/objects/arrow.hpp>

namespace legbot_rviz_tools {
class SurfacePoseTool : public rviz_common::Tool {
public:
  explicit SurfacePoseTool(bool estimate) : estimate_(estimate) {
    shortcut_key_ = estimate ? 'i' : 'g';
    offset_ = new rviz_common::properties::FloatProperty(
      "Height offset", 0.0f, "Meters above the picked 3D surface.", getPropertyContainer());
  }
  void onInitialize() override {
    setName(estimate_ ? "3D Pose Estimate" : "3D Nav Goal");
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    if (estimate_) {
      estimate_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    } else {
      goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pct/nav_goal", 10);
    }
    arrow_ = std::make_unique<rviz_rendering::Arrow>(scene_manager_, nullptr, 1.0f, 0.12f, 0.35f, 0.25f);
    arrow_->setColor(estimate_ ? 0.2f : 1.0f, 0.7f, 0.1f, 1.0f);
    arrow_->getSceneNode()->setVisible(false);
  }
  void activate() override {
    dragging_ = false;
    setStatus(estimate_
      ? "Select robot position on a 3D surface and drag heading. Requires an /initialpose localization consumer; Gazebo truth does not use this."
      : "Click a walkable 3D surface, drag heading, release to navigate. Height follows the surface. Esc cancels.");
  }
  void deactivate() override {
    dragging_ = false;
    if (arrow_) arrow_->getSceneNode()->setVisible(false);
  }
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel *) override {
    if (event->key() == Qt::Key_Escape) {deactivate(); return Render | Finished;}
    return 0;
  }
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override {
    if (event.rightDown()) {deactivate(); return Render | Finished;}
    if (event.leftDown()) {
      if (!context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position_)) {
        setStatus("No 3D surface under cursor. Click the floor point cloud, not empty space.");
        return Render;
      }
      if (!std::isfinite(position_.x) || !std::isfinite(position_.y) || !std::isfinite(position_.z)) return 0;
      position_.z += offset_->getFloat();
      frame_ = context_->getFixedFrame().toStdString();
      dragging_ = true;
      yaw_ = 0.0;
      arrow_->setPosition(position_);
      arrow_->setDirection(Ogre::Vector3::UNIT_X);
      arrow_->getSceneNode()->setVisible(true);
    }
    if (dragging_ && event.type == QEvent::MouseMove) {
      auto * window = event.panel->getRenderWindow();
      auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(window);
      auto * camera = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(window);
      auto ray = camera->getCameraToViewportRay(
        static_cast<float>(event.x) / viewport->getActualWidth(),
        static_cast<float>(event.y) / viewport->getActualHeight());
      auto intersection = ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z, position_));
      if (intersection.first) {
        auto delta = ray.getPoint(intersection.second) - position_;
        if (delta.squaredLength() > 1e-6) yaw_ = std::atan2(delta.y, delta.x);
        arrow_->setDirection(Ogre::Vector3(std::cos(yaw_), std::sin(yaw_), 0.0f));
      }
    }
    if (dragging_ && event.leftUp()) {
      if (frame_ != context_->getFixedFrame().toStdString()) {
        setStatus("Fixed frame changed while selecting; select the target again.");
        deactivate(); return Render | Finished;
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_;
      pose.header.stamp = node_->now();
      pose.pose.position.x = position_.x;
      pose.pose.position.y = position_.y;
      pose.pose.position.z = position_.z;
      pose.pose.orientation.z = std::sin(yaw_ / 2.0);
      pose.pose.orientation.w = std::cos(yaw_ / 2.0);
      if (estimate_) {
        if (estimate_pub_->get_subscription_count() == 0) {
          setStatus("No localization consumer for /initialpose. Current Gazebo truth needs no pose estimate.");
          RCLCPP_WARN(node_->get_logger(), "3D Pose Estimate not sent: no /initialpose consumer (Gazebo truth needs no initialization).");
        } else {
          geometry_msgs::msg::PoseWithCovarianceStamped msg;
          msg.header = pose.header; msg.pose.pose = pose.pose;
          msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[14] = 0.25;
          msg.pose.covariance[21] = msg.pose.covariance[28] = msg.pose.covariance[35] = 0.07;
          estimate_pub_->publish(msg); setStatus("3D pose estimate sent to /initialpose.");
        }
      } else if (goal_pub_->get_subscription_count() == 0) {
        setStatus("No 3D planner connected. Start simulation.sh --3d.");
        RCLCPP_WARN(node_->get_logger(), "3D Nav Goal not sent: no /pct/nav_goal subscriber.");
      } else {
        goal_pub_->publish(pose);
        setStatus("3D goal sent; see the target label for planning result.");
      }
      deactivate(); return Render | Finished;
    }
    return Render;
  }
private:
  bool estimate_, dragging_ = false;
  double yaw_ = 0.0;
  std::string frame_;
  Ogre::Vector3 position_;
  rviz_common::properties::FloatProperty * offset_;
  std::unique_ptr<rviz_rendering::Arrow> arrow_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimate_pub_;
};
class NavGoal3D : public SurfacePoseTool {public: NavGoal3D() : SurfacePoseTool(false) {}};
class PoseEstimate3D : public SurfacePoseTool {public: PoseEstimate3D() : SurfacePoseTool(true) {}};
}
PLUGINLIB_EXPORT_CLASS(legbot_rviz_tools::NavGoal3D, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(legbot_rviz_tools::PoseEstimate3D, rviz_common::Tool)
