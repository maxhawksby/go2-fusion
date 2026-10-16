#pragma once

#include <thread>

#include <QProcess>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

class QLabel;
class QPlainTextEdit;
class QPushButton;

namespace go2_rviz_plugins
{

// Runs `ros2 run go2_auto_mapper explorer` as a child process and surfaces
// start/stop controls plus its log output. Two buttons call std_srvs/Trigger
// services through a small rclcpp node of the panel's own, spun on a
// background thread like MapViewerPanel's:
//   - Scan: the explorer's /frontier_explorer/scan (a 360 degree turn so
//     SLAM can re-place the robot).
//   - Reset maps: lidar_processor's /lidar_to_pointcloud/reset (clears the
//     aggregated cloud behind /pointcloud/aggregated and 3d_map.ply). The
//     same button also restarts slam_toolbox for a fresh 2D map (see
//     onResetMapClicked: Humble's slam_toolbox has no reset service).
// See go2_auto_mapper/README.md for what the explorer does and how to stop
// it safely by hand if this panel isn't used.
class AutoMapperPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AutoMapperPanel(QWidget * parent = nullptr);
  ~AutoMapperPanel() override;

signals:
  // Emitted from the ROS spin thread; Qt queues it onto the GUI thread.
  void serviceReplied(const QString & label, bool success, const QString & message);

private slots:
  void onScanClicked();
  void onResetMapClicked();
  void onServiceReplied(const QString & label, bool success, const QString & message);
  void onStartStopClicked();
  void onReadyReadStandardOutput();
  void onReadyReadStandardError();
  void onProcessStarted();
  void onProcessFinished(int exit_code, QProcess::ExitStatus exit_status);
  void onProcessErrorOccurred(QProcess::ProcessError error);
  void onForceKillTimeout();

private:
  using TriggerClient = rclcpp::Client<std_srvs::srv::Trigger>;

  void setStatus(const QString & text);
  void appendLog(const QString & text);
  // Calls `client`, disabling `button` until the reply (or a timeout).
  // `label` names the action in the log and identifies the reply.
  void callTrigger(
    const TriggerClient::SharedPtr & client, QPushButton * button,
    const QString & label, const QString & not_running_hint);
  QPushButton * buttonFor(const QString & label) const;

  QProcess * process_;
  QPushButton * start_stop_button_;
  QPushButton * scan_button_;
  QPushButton * reset_map_button_;
  QLabel * status_label_;
  QPlainTextEdit * log_view_;
  bool stopping_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  TriggerClient::SharedPtr scan_client_;
  TriggerClient::SharedPtr reset_map_client_;
};

}  // namespace go2_rviz_plugins
