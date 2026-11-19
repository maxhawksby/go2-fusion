#pragma once

#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <QImage>
#include <QLabel>
#include <QPointF>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class QMouseEvent;
class QPaintEvent;
class QPushButton;

namespace go2_rviz_plugins
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// Where the currently-held map sits inside the widget, recomputed on every
// paint/click since the widget (and therefore the fit) can be resized at any
// time between them.
struct MapLayout
{
  bool valid{false};
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  double scale{1.0};
  double offset_x{0.0};
  double offset_y{0.0};
};

// Draws the occupancy grid with QPainter instead of through RViz's OGRE
// scene, so it isn't affected by the GLSL/Mesa shader crash that hits
// rviz_default_plugins' own Map display under software rendering (see
// go2_robot_sdk/config/single_robot_conf.rviz, where that display is
// disabled for the same reason). Emits worldClicked(x, y) on left-click
// inside the map bounds.
class MapCanvas : public QWidget
{
  Q_OBJECT

public:
  explicit MapCanvas(QWidget * parent = nullptr);

  // Takes ownership of building its own display image; safe to call from
  // the Qt (GUI) thread only.
  void setMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void setRobotPose(bool valid, double x, double y);

signals:
  void worldClicked(double x, double y);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;

private:
  MapLayout computeLayout() const;
  bool imageToWorld(const MapLayout & layout, double mouse_x, double mouse_y, double & wx, double & wy) const;
  QPointF worldToImage(const MapLayout & layout, double wx, double wy) const;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  QImage grid_image_;

  bool have_robot_pose_{false};
  double robot_x_{0.0};
  double robot_y_{0.0};
};

// Owns a dedicated rclcpp::Node (spun on a background thread, separate from
// RViz's own ROS plumbing) so it can subscribe to /map, look up the
// map->base_link transform, and drive a NavigateToPose action client, the
// same responsibilities go2_map_viewer_cpp's standalone node has.
class MapViewerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MapViewerPanel(QWidget * parent = nullptr);
  ~MapViewerPanel() override;

private slots:
  void onWorldClicked(double x, double y);
  void onCancelClicked();
  void onMapUpdated();
  void onRobotPoseUpdated();
  void onStatusUpdated();

signals:
  // Emitted from ROS callbacks (background thread); connected with Qt's
  // default auto connection, which queues onto the GUI thread since the
  // emitting thread differs from this QObject's thread affinity.
  void mapUpdated();
  void robotPoseUpdated();
  void statusUpdated();

private:
  void spinNode();
  void sendGoal(double x, double y);
  void cancelGoal();
  void setStatus(const QString & text);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<bool> stop_spinning_{false};
  std::thread spin_thread_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr pose_timer_;

  std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr pending_map_;

  std::mutex pose_mutex_;
  bool pending_pose_valid_{false};
  double pending_pose_x_{0.0};
  double pending_pose_y_{0.0};

  std::mutex status_mutex_;
  QString pending_status_;

  // Goal lifecycle: mirrors go2_map_viewer_cpp's map_viewer_node.cpp. A
  // generation counter stops a late/stale goal response or result from
  // clobbering the handle for whatever goal is actually current, and
  // cancel_requested_ remembers a cancel pressed before a goal was even
  // accepted so it isn't silently dropped.
  std::mutex goal_mutex_;
  GoalHandleNav::SharedPtr current_goal_handle_;
  uint64_t goal_generation_{0};
  bool cancel_requested_{false};

  // Closing the panel cancels its goals (see the destructor). Goals sent but
  // not yet accepted or rejected are counted, since they have no handle yet;
  // closing_ makes the response callback cancel them, into close_cancels_.
  using CancelFuture =
    std::shared_future<rclcpp_action::Client<NavigateToPose>::CancelResponse::SharedPtr>;
  std::condition_variable goals_answered_;
  int goals_awaiting_answer_{0};
  bool closing_{false};
  std::vector<CancelFuture> close_cancels_;

  MapCanvas * canvas_;
  QLabel * status_label_;
  QPushButton * cancel_button_;
};

}  // namespace go2_rviz_plugins
