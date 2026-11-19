#include "go2_rviz_plugins/map_viewer_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>
#include <QVBoxLayout>

namespace go2_rviz_plugins
{

namespace
{
constexpr double kHintBarPx = 24.0;

// How long the spin thread waits for work before checking whether to stop.
constexpr auto kSpinPollPeriod = std::chrono::milliseconds(100);

// How long closing the panel waits for Nav2 to answer its goals and cancels.
constexpr auto kCancelOnCloseTimeout = std::chrono::seconds(2);

// 0 = free -> white, 100 = occupied -> black, -1 = unknown -> mid gray.
QRgb cellColor(int8_t value)
{
  if (value < 0) {
    return qRgb(120, 120, 120);
  }
  int shade = 255 - static_cast<int>((static_cast<double>(value) / 100.0) * 255.0);
  shade = std::max(0, std::min(255, shade));
  return qRgb(shade, shade, shade);
}
}  // namespace

MapCanvas::MapCanvas(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(200, 200);
  setMouseTracking(false);
}

void MapCanvas::setMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  const int width = static_cast<int>(msg->info.width);
  const int height = static_cast<int>(msg->info.height);
  const size_t expected =
    static_cast<size_t>(std::max(width, 0)) * static_cast<size_t>(std::max(height, 0));
  if (width <= 0 || height <= 0 || msg->info.resolution <= 0.0 || msg->data.size() < expected) {
    return;
  }

  // Grid row 0 is the bottom-most row in the map frame; QImage row 0 is the
  // top, so rows are flipped on the way in (same convention as the SFML
  // viewer's texture build in go2_map_viewer_cpp).
  QImage image(width, height, QImage::Format_RGB32);
  for (int y = 0; y < height; ++y) {
    const int flipped_y = height - 1 - y;
    auto * line = reinterpret_cast<QRgb *>(image.scanLine(flipped_y));
    for (int x = 0; x < width; ++x) {
      line[x] = cellColor(msg->data[static_cast<size_t>(y) * width + x]);
    }
  }

  map_ = msg;
  grid_image_ = image;
  update();
}

void MapCanvas::setRobotPose(bool valid, double x, double y)
{
  have_robot_pose_ = valid;
  robot_x_ = x;
  robot_y_ = y;
  update();
}

MapLayout MapCanvas::computeLayout() const
{
  MapLayout layout;
  if (!map_ || grid_image_.isNull()) {
    return layout;
  }

  const double view_w = static_cast<double>(width());
  const double avail_h = std::max(1.0, static_cast<double>(height()) - kHintBarPx);
  if (view_w < 1.0 || avail_h < 1.0) {
    return layout;
  }

  const double scale = std::min(
    view_w / static_cast<double>(grid_image_.width()),
    avail_h / static_cast<double>(grid_image_.height()));
  if (!(scale > 0.0)) {
    return layout;
  }

  layout.valid = true;
  layout.width = grid_image_.width();
  layout.height = grid_image_.height();
  layout.resolution = map_->info.resolution;
  layout.origin_x = map_->info.origin.position.x;
  layout.origin_y = map_->info.origin.position.y;
  layout.scale = scale;
  layout.offset_x = (view_w - layout.width * scale) * 0.5;
  layout.offset_y = (avail_h - layout.height * scale) * 0.5;
  return layout;
}

bool MapCanvas::imageToWorld(
  const MapLayout & layout, double mouse_x, double mouse_y, double & wx, double & wy) const
{
  if (!layout.valid) {
    return false;
  }
  const double ix = (mouse_x - layout.offset_x) / layout.scale;
  const double iy = (mouse_y - layout.offset_y) / layout.scale;
  if (ix < 0.0 || iy < 0.0 || ix >= layout.width || iy >= layout.height) {
    return false;
  }
  wx = layout.origin_x + ix * layout.resolution;
  wy = layout.origin_y + layout.height * layout.resolution - iy * layout.resolution;
  return true;
}

QPointF MapCanvas::worldToImage(const MapLayout & layout, double wx, double wy) const
{
  const double ix = (wx - layout.origin_x) / layout.resolution;
  const double iy = layout.height - (wy - layout.origin_y) / layout.resolution;
  return QPointF(layout.offset_x + ix * layout.scale, layout.offset_y + iy * layout.scale);
}

void MapCanvas::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.fillRect(rect(), QColor(30, 30, 30));

  const MapLayout layout = computeLayout();
  if (!layout.valid) {
    painter.setPen(Qt::white);
    painter.drawText(
      rect(), Qt::AlignCenter, map_ ? "Waiting for a valid map..." : "Waiting for map...");
    return;
  }

  const QRectF target(
    layout.offset_x, layout.offset_y,
    layout.width * layout.scale, layout.height * layout.scale);
  painter.drawImage(target, grid_image_);

  if (have_robot_pose_) {
    const QPointF p = worldToImage(layout, robot_x_, robot_y_);
    painter.setBrush(Qt::red);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(p, 6.0, 6.0);
  }

  painter.setPen(Qt::yellow);
  painter.drawText(
    QRectF(4, height() - kHintBarPx, width() - 8, kHintBarPx),
    Qt::AlignVCenter, "Click: send Nav2 goal");
}

void MapCanvas::mousePressEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton) {
    return;
  }
  const MapLayout layout = computeLayout();
  double wx = 0.0;
  double wy = 0.0;
  if (imageToWorld(layout, event->pos().x(), event->pos().y(), wx, wy)) {
    emit worldClicked(wx, wy);
  }
}

MapViewerPanel::MapViewerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  canvas_ = new MapCanvas(this);
  status_label_ = new QLabel("Starting...", this);
  cancel_button_ = new QPushButton("Cancel Goal", this);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(canvas_, /*stretch=*/1);
  layout->addWidget(status_label_);
  layout->addWidget(cancel_button_);
  setLayout(layout);

  connect(canvas_, &MapCanvas::worldClicked, this, &MapViewerPanel::onWorldClicked);
  connect(cancel_button_, &QPushButton::clicked, this, &MapViewerPanel::onCancelClicked);
  connect(this, &MapViewerPanel::mapUpdated, this, &MapViewerPanel::onMapUpdated);
  connect(this, &MapViewerPanel::robotPoseUpdated, this, &MapViewerPanel::onRobotPoseUpdated);
  connect(this, &MapViewerPanel::statusUpdated, this, &MapViewerPanel::onStatusUpdated);

  node_ = rclcpp::Node::make_shared("go2_rviz_map_viewer_panel");

  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        pending_map_ = msg;
      }
      emit mapUpdated();
    });

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // spin_thread=false: node_ is already spun by our own executor_ below.
  // The default (true) makes TransformListener create its own internal
  // executor for the /tf subscription's callback group on the same node,
  // which conflicts with executor_.add_node(node_) further down.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  pose_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(200),
    [this]() {
      double x = 0.0;
      double y = 0.0;
      bool valid = false;
      try {
        auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        x = t.transform.translation.x;
        y = t.transform.translation.y;
        valid = true;
      } catch (const tf2::TransformException &) {
        valid = false;
      }
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pending_pose_valid_ = valid;
        pending_pose_x_ = x;
        pending_pose_y_ = y;
      }
      emit robotPoseUpdated();
    });

  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() {spinNode();});

  setStatus("Waiting for map...");
}

MapViewerPanel::~MapViewerPanel()
{
  // Destroying the action client doesn't stop Nav2: a goal sent from this
  // panel would keep the robot walking with no Cancel button left. Cancel
  // it while the spin thread is still running to carry the requests out.
  const auto deadline = std::chrono::steady_clock::now() + kCancelOnCloseTimeout;
  std::vector<CancelFuture> cancels;
  {
    std::unique_lock<std::mutex> lock(goal_mutex_);
    closing_ = true;
    if (current_goal_handle_) {
      close_cancels_.push_back(nav_client_->async_cancel_goal(current_goal_handle_));
      current_goal_handle_.reset();
    }
    // A goal not accepted yet has no handle: wait for Nav2's answer. The
    // response callback cancels it if accepted (closing_ is set).
    const bool answered = goals_answered_.wait_until(
      lock, deadline, [this]() {return goals_awaiting_answer_ == 0;});
    if (!answered) {
      // Nav2 may still accept it later, with nothing left to cancel it. This
      // also cancels other nodes' goals: stopping the robot is the safe side.
      close_cancels_.push_back(nav_client_->async_cancel_all_goals());
    }
    cancels = close_cancels_;
  }
  // Its own deadline: the answer wait above may have used up the first one.
  // Known limit: a goal Nav2 receives only after the cancel-all is not
  // covered; that needs Nav2 to be failing already (no answer in 2 s).
  const auto cancel_deadline = std::chrono::steady_clock::now() + kCancelOnCloseTimeout;
  for (const auto & cancel : cancels) {
    cancel.wait_until(cancel_deadline);
  }

  stop_spinning_ = true;
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void MapViewerPanel::spinNode()
{
  // Not executor_.spin(): a cancel() that lands before spin() starts is lost,
  // and the destructor's join() would then wait forever.
  while (!stop_spinning_ && rclcpp::ok()) {
    executor_.spin_once(kSpinPollPeriod);
  }
}

void MapViewerPanel::setStatus(const QString & text)
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    pending_status_ = text;
  }
  emit statusUpdated();
}

void MapViewerPanel::onMapUpdated()
{
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    msg = pending_map_;
  }
  if (msg) {
    canvas_->setMap(msg);
  }
}

void MapViewerPanel::onRobotPoseUpdated()
{
  bool valid = false;
  double x = 0.0;
  double y = 0.0;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    valid = pending_pose_valid_;
    x = pending_pose_x_;
    y = pending_pose_y_;
  }
  canvas_->setRobotPose(valid, x, y);
}

void MapViewerPanel::onStatusUpdated()
{
  QString text;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    text = pending_status_;
  }
  status_label_->setText(text);
}

void MapViewerPanel::onWorldClicked(double x, double y)
{
  sendGoal(x, y);
}

void MapViewerPanel::onCancelClicked()
{
  cancelGoal();
}

void MapViewerPanel::sendGoal(double x, double y)
{
  if (!nav_client_->action_server_is_ready()) {
    setStatus("Nav2 action server not available.");
    return;
  }

  uint64_t my_generation;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (current_goal_handle_) {
      nav_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
    }
    my_generation = ++goal_generation_;
    cancel_requested_ = false;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->get_clock()->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.orientation.w = 1.0;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, my_generation](const GoalHandleNav::SharedPtr & handle) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      --goals_awaiting_answer_;
      goals_answered_.notify_all();
      if (closing_) {
        // The panel is closing: nothing may keep driving, whatever goal it is.
        if (handle) {
          close_cancels_.push_back(nav_client_->async_cancel_goal(handle));
        }
        return;
      }
      if (my_generation != goal_generation_) {
        return;
      }
      if (!handle) {
        current_goal_handle_.reset();
        setStatus("Goal was rejected.");
        return;
      }
      current_goal_handle_ = handle;
      if (cancel_requested_) {
        nav_client_->async_cancel_goal(handle);
        current_goal_handle_.reset();
        cancel_requested_ = false;
        setStatus("Cancelled goal that was pending cancellation.");
      } else {
        setStatus("Goal accepted.");
      }
    };
  send_goal_options.result_callback =
    [this, my_generation](const GoalHandleNav::WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (my_generation != goal_generation_) {
          // A replaced goal finishing (usually "cancelled") says nothing
          // about the goal that is current now.
          return;
        }
        current_goal_handle_.reset();
      }
      const char * outcome = "UNKNOWN";
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: outcome = "reached"; break;
        case rclcpp_action::ResultCode::ABORTED: outcome = "aborted"; break;
        case rclcpp_action::ResultCode::CANCELED: outcome = "cancelled"; break;
        default: break;
      }
      setStatus(QString("Navigation %1.").arg(outcome));
    };

  setStatus(QString("Sending goal: x=%1 y=%2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    ++goals_awaiting_answer_;
  }
  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void MapViewerPanel::cancelGoal()
{
  GoalHandleNav::SharedPtr handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    handle = current_goal_handle_;
    current_goal_handle_.reset();
    if (!handle) {
      cancel_requested_ = true;
    }
  }
  if (handle) {
    nav_client_->async_cancel_goal(handle);
    setStatus("Cancelling goal.");
  } else {
    setStatus("No active goal (cancel will apply if one is pending acceptance).");
  }
}

}  // namespace go2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(go2_rviz_plugins::MapViewerPanel, rviz_common::Panel)
