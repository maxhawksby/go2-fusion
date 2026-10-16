#include "go2_rviz_plugins/auto_mapper_panel.hpp"

#include <csignal>
#include <unistd.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace go2_rviz_plugins
{

namespace
{
// How long to wait after a graceful SIGINT (the Ctrl+C-equivalent path
// explorer.py's own shutdown handling expects, so it cancels any in-flight
// Nav2 goal) before giving up and force-killing the process.
constexpr int kGracefulStopTimeoutMs = 5000;

// Re-enable a service button if the node never answers the request.
constexpr int kServiceReplyTimeoutMs = 5000;

constexpr char kScanService[] = "/frontier_explorer/scan";
constexpr char kResetMapService[] = "/lidar_to_pointcloud/reset";

constexpr char kScanLabel[] = "Scan";
constexpr char kResetMapLabel[] = "Reset maps";

// Humble's slam_toolbox has no reset service. robot.launch.py starts it with
// respawn=True, so stopping it (SIGINT, a clean shutdown) makes launch start
// a fresh one with an empty map. RViz runs in the same container as the
// stack, so pkill reaches it.
constexpr char kSlamProcessPattern[] = "async_slam_toolbox_node";
}  // namespace

AutoMapperPanel::AutoMapperPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  start_stop_button_ = new QPushButton("Start", this);
  scan_button_ = new QPushButton(kScanLabel, this);
  scan_button_->setToolTip(
    "Turn the robot 360 degrees so SLAM can re-place it on the map. "
    "Pauses the current goal; needs the explorer running.");
  reset_map_button_ = new QPushButton(kResetMapLabel, this);
  reset_map_button_->setToolTip(
    "Start both maps over from where the robot stands: restarts SLAM (the 2D "
    "/map the auto mapper explores) and clears the accumulated 3D point "
    "cloud (/pointcloud/aggregated, 3d_map.ply). Asks to confirm first.");
  status_label_ = new QLabel("Stopped", this);
  log_view_ = new QPlainTextEdit(this);
  log_view_->setReadOnly(true);
  log_view_->setMaximumBlockCount(1000);

  auto * buttons = new QHBoxLayout();
  buttons->addWidget(start_stop_button_);
  buttons->addWidget(scan_button_);
  buttons->addWidget(reset_map_button_);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(status_label_);
  layout->addLayout(buttons);
  layout->addWidget(log_view_, /*stretch=*/1);
  setLayout(layout);

  node_ = rclcpp::Node::make_shared("go2_rviz_auto_mapper_panel");
  scan_client_ = node_->create_client<std_srvs::srv::Trigger>(kScanService);
  reset_map_client_ = node_->create_client<std_srvs::srv::Trigger>(kResetMapService);
  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() {executor_.spin();});

  connect(scan_button_, &QPushButton::clicked, this, &AutoMapperPanel::onScanClicked);
  connect(reset_map_button_, &QPushButton::clicked, this, &AutoMapperPanel::onResetMapClicked);
  connect(this, &AutoMapperPanel::serviceReplied, this, &AutoMapperPanel::onServiceReplied);

  process_ = new QProcess(this);
  process_->setProcessChannelMode(QProcess::SeparateChannels);

  connect(start_stop_button_, &QPushButton::clicked, this, &AutoMapperPanel::onStartStopClicked);
  connect(process_, &QProcess::readyReadStandardOutput, this, &AutoMapperPanel::onReadyReadStandardOutput);
  connect(process_, &QProcess::readyReadStandardError, this, &AutoMapperPanel::onReadyReadStandardError);
  connect(process_, &QProcess::started, this, &AutoMapperPanel::onProcessStarted);
  connect(
    process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this, &AutoMapperPanel::onProcessFinished);
  connect(process_, &QProcess::errorOccurred, this, &AutoMapperPanel::onProcessErrorOccurred);
}

AutoMapperPanel::~AutoMapperPanel()
{
  // Stop the spin thread first: a late service reply must not call back into
  // a panel that's being destroyed.
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  if (process_->state() != QProcess::NotRunning) {
    ::kill(static_cast<pid_t>(process_->processId()), SIGINT);
    process_->waitForFinished(kGracefulStopTimeoutMs);
    if (process_->state() != QProcess::NotRunning) {
      process_->kill();
      process_->waitForFinished(1000);
    }
  }
}

void AutoMapperPanel::setStatus(const QString & text)
{
  status_label_->setText(text);
}

void AutoMapperPanel::appendLog(const QString & text)
{
  for (const QString & line : text.split('\n', Qt::SkipEmptyParts)) {
    log_view_->appendPlainText(line);
  }
}

void AutoMapperPanel::onStartStopClicked()
{
  if (process_->state() == QProcess::NotRunning) {
    stopping_ = false;
    log_view_->clear();
    setStatus("Starting...");
    start_stop_button_->setEnabled(false);
    // Inherits this process's environment, which is already ROS-sourced
    // since rviz2 itself was launched through ros2 launch.
    process_->start("ros2", {"run", "go2_auto_mapper", "explorer"});
  } else {
    stopping_ = true;
    setStatus("Stopping...");
    start_stop_button_->setEnabled(false);
    // SIGINT (not QProcess::terminate()'s SIGTERM) so explorer.py's own
    // KeyboardInterrupt/ExternalShutdownException handling runs and cancels
    // any in-flight Nav2 goal before exiting, matching the stop behavior
    // documented in go2_auto_mapper/README.md.
    ::kill(static_cast<pid_t>(process_->processId()), SIGINT);
    QTimer::singleShot(kGracefulStopTimeoutMs, this, &AutoMapperPanel::onForceKillTimeout);
  }
}

void AutoMapperPanel::onScanClicked()
{
  callTrigger(
    scan_client_, scan_button_, kScanLabel,
    "the explorer isn't running. Press Start first.");
}

void AutoMapperPanel::onResetMapClicked()
{
  const auto answer = QMessageBox::question(
    this, "Reset maps",
    "Throw away the 2D SLAM map and the 3D point cloud, and start both over "
    "from where the robot is now?\n\n"
    "Navigation pauses for a few seconds while SLAM restarts.");
  if (answer != QMessageBox::Yes) {
    return;
  }

  // 2D: stop slam_toolbox; launch respawns it with an empty map.
  const int pkill_code = QProcess::execute("pkill", {"-INT", "-f", kSlamProcessPattern});
  if (pkill_code == 0) {
    appendLog("[panel] Reset maps: SLAM restarting with a fresh 2D map.");
  } else {
    appendLog(
      QString("[panel] Reset maps: couldn't stop SLAM (pkill exit code %1); "
      "is the robot stack running in this container?").arg(pkill_code));
  }

  // 3D: clear lidar_processor's accumulated cloud.
  callTrigger(
    reset_map_client_, reset_map_button_, kResetMapLabel,
    "lidar_to_pointcloud isn't running, so the 3D cloud wasn't cleared.");
}

void AutoMapperPanel::callTrigger(
  const TriggerClient::SharedPtr & client, QPushButton * button,
  const QString & label, const QString & not_running_hint)
{
  if (!client->service_is_ready()) {
    appendLog(QString("[panel] %1: %2").arg(label, not_running_hint));
    return;
  }

  button->setEnabled(false);
  QTimer::singleShot(kServiceReplyTimeoutMs, button, [button]() {button->setEnabled(true);});

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
    request,
    [this, label](TriggerClient::SharedFuture future) {
      const auto response = future.get();
      emit serviceReplied(label, response->success, QString::fromStdString(response->message));
    });
}

QPushButton * AutoMapperPanel::buttonFor(const QString & label) const
{
  return label == kScanLabel ? scan_button_ : reset_map_button_;
}

void AutoMapperPanel::onServiceReplied(
  const QString & label, bool success, const QString & message)
{
  buttonFor(label)->setEnabled(true);
  appendLog(QString("[panel] %1: %2").arg(label, success ? message : "refused, " + message));
}

void AutoMapperPanel::onForceKillTimeout()
{
  if (stopping_ && process_->state() != QProcess::NotRunning) {
    appendLog("[panel] No clean exit after SIGINT, force-killing.");
    process_->kill();
  }
}

void AutoMapperPanel::onReadyReadStandardOutput()
{
  appendLog(QString::fromUtf8(process_->readAllStandardOutput()));
}

void AutoMapperPanel::onReadyReadStandardError()
{
  appendLog(QString::fromUtf8(process_->readAllStandardError()));
}

void AutoMapperPanel::onProcessStarted()
{
  setStatus(QString("Running (pid %1)").arg(process_->processId()));
  start_stop_button_->setText("Stop");
  start_stop_button_->setEnabled(true);
}

void AutoMapperPanel::onProcessFinished(int exit_code, QProcess::ExitStatus exit_status)
{
  if (exit_status == QProcess::CrashExit) {
    setStatus(QString("Stopped (signal, exit code %1)").arg(exit_code));
  } else {
    setStatus(QString("Stopped (exit code %1)").arg(exit_code));
  }
  start_stop_button_->setText("Start");
  start_stop_button_->setEnabled(true);
  stopping_ = false;
}

void AutoMapperPanel::onProcessErrorOccurred(QProcess::ProcessError error)
{
  if (error == QProcess::FailedToStart) {
    setStatus("Failed to start (is go2_auto_mapper installed?)");
    start_stop_button_->setText("Start");
    start_stop_button_->setEnabled(true);
  }
}

}  // namespace go2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(go2_rviz_plugins::AutoMapperPanel, rviz_common::Panel)
