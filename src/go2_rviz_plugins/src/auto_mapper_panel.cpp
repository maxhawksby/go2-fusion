#include "go2_rviz_plugins/auto_mapper_panel.hpp"

#include <csignal>
#include <string>
#include <unistd.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPixmap>
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

// How long the spin thread waits for work before checking whether to stop.
constexpr auto kSpinPollPeriod = std::chrono::milliseconds(100);

// Width the VLM's picture is shown at, in pixels.
constexpr int kVlmImageWidth = 320;

// An installed go2_auto_mapper executable, or an empty string if it can't be found.
// Starting it directly (not through `ros2 run`) makes it the panel's own child:
// Stop's SIGINT reaches it, and its parent-death signal fires when RViz exits.
// Through `ros2 run`, both went to the wrapper, which outlives RViz.
QString mapperExecutable(const std::string & name)
{
  try {
    const std::string prefix = ament_index_cpp::get_package_prefix("go2_auto_mapper");
    const QString path = QString::fromStdString(prefix + "/lib/go2_auto_mapper/" + name);
    return QFileInfo(path).isExecutable() ? path : QString();
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    return QString();
  }
}
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

  vlm_button_ = new QPushButton("VLM view", this);
  vlm_button_->setCheckable(true);
  vlm_button_->setToolTip(
    "Ask the VLM to describe Oak's camera every few seconds and show the answer. "
    "Doesn't move the robot. Needs VLM_URL and VLM_MODEL pointing at a running server.");
  buttons->addWidget(vlm_button_);
  vlm_image_ = new QLabel(this);
  vlm_image_->setAlignment(Qt::AlignCenter);
  vlm_image_->setVisible(false);
  vlm_text_ = new QLabel(this);
  vlm_text_->setWordWrap(true);
  vlm_text_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  vlm_text_->setVisible(false);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(status_label_);
  layout->addLayout(buttons);
  layout->addWidget(vlm_image_);
  layout->addWidget(vlm_text_);
  layout->addWidget(log_view_, /*stretch=*/1);
  setLayout(layout);

  node_ = rclcpp::Node::make_shared("go2_rviz_auto_mapper_panel");
  scan_client_ = node_->create_client<std_srvs::srv::Trigger>(kScanService);
  reset_map_client_ = node_->create_client<std_srvs::srv::Trigger>(kResetMapService);
  // Called on the spin thread; the signals queue onto the GUI thread.
  const auto latched = rclcpp::QoS(1).transient_local();
  vlm_view_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/vlm/view", latched,
    [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
      emit vlmViewReceived(
        QByteArray(reinterpret_cast<const char *>(msg->data.data()), msg->data.size()));
    });
  vlm_description_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/vlm/description", latched,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      emit vlmDescriptionReceived(QString::fromStdString(msg->data));
    });
  executor_.add_node(node_);
  // Not executor_.spin(): a cancel() that lands before spin() starts is lost,
  // and the destructor's join() would then wait forever.
  spin_thread_ = std::thread(
    [this]() {
      while (!stop_spinning_ && rclcpp::ok()) {
        executor_.spin_once(kSpinPollPeriod);
      }
    });

  connect(scan_button_, &QPushButton::clicked, this, &AutoMapperPanel::onScanClicked);
  connect(reset_map_button_, &QPushButton::clicked, this, &AutoMapperPanel::onResetMapClicked);
  connect(this, &AutoMapperPanel::serviceReplied, this, &AutoMapperPanel::onServiceReplied);

  vlm_process_ = new QProcess(this);
  connect(vlm_button_, &QPushButton::clicked, this, &AutoMapperPanel::onVlmViewClicked);
  connect(this, &AutoMapperPanel::vlmViewReceived, this, &AutoMapperPanel::onVlmViewReceived);
  connect(
    this, &AutoMapperPanel::vlmDescriptionReceived,
    this, &AutoMapperPanel::onVlmDescriptionReceived);

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
  stop_spinning_ = true;
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  if (vlm_process_->state() != QProcess::NotRunning) {
    ::kill(static_cast<pid_t>(vlm_process_->processId()), SIGINT);
    if (!vlm_process_->waitForFinished(1000)) {
      vlm_process_->kill();
      vlm_process_->waitForFinished(1000);
    }
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
    const QString explorer = mapperExecutable("explorer");
    if (explorer.isEmpty()) {
      appendLog(
        "[panel] go2_auto_mapper's explorer not found directly; using `ros2 run`. "
        "Stop and closing RViz may then leave it running (see the README).");
      process_->start("ros2", {"run", "go2_auto_mapper", "explorer"});
    } else {
      process_->start(explorer, QStringList());
    }
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

void AutoMapperPanel::onVlmViewClicked()
{
  const bool on = vlm_button_->isChecked();
  vlm_image_->setVisible(on);
  vlm_text_->setVisible(on);
  if (!on) {
    if (vlm_process_->state() != QProcess::NotRunning) {
      ::kill(static_cast<pid_t>(vlm_process_->processId()), SIGINT);
    }
    return;
  }
  const QString vlm_view = mapperExecutable("vlm_view");
  if (vlm_view.isEmpty()) {
    vlm_text_->setText("vlm_view not found: rebuild go2_auto_mapper.");
    return;
  }
  vlm_text_->setText("Asking the VLM...");
  vlm_process_->start(vlm_view, QStringList());
}

void AutoMapperPanel::onVlmViewReceived(const QByteArray & jpeg)
{
  QPixmap picture;
  if (picture.loadFromData(jpeg, "JPEG")) {
    vlm_image_->setPixmap(picture.scaledToWidth(kVlmImageWidth, Qt::SmoothTransformation));
  }
}

void AutoMapperPanel::onVlmDescriptionReceived(const QString & text)
{
  vlm_text_->setText(text);
}

void AutoMapperPanel::onScanClicked()
{
  callTrigger(
    scan_client_, scan_button_, kScanLabel,
    "the explorer isn't running. Press Start first.");
}

void AutoMapperPanel::onResetMapClicked()
{
  // The explorer's goals, failed spots and frontiers belong to the old map,
  // and a scan in progress keeps turning without one. Stopping it first
  // cancels its motion and confirms that before it exits.
  if (process_->state() != QProcess::NotRunning) {
    appendLog("[panel] Reset maps: stop the auto mapper first (Stop), then reset.");
    return;
  }

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

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  const int64_t request_id = client->async_send_request(
    request,
    [this, label](TriggerClient::SharedFuture future) {
      const auto response = future.get();
      emit serviceReplied(label, response->success, QString::fromStdString(response->message));
    }).request_id;
  pending_requests_[label] = request_id;

  // No reply in time: drop the request so the client doesn't keep it (and
  // its callback) forever, and give the button back.
  QTimer::singleShot(
    kServiceReplyTimeoutMs, button, [this, client, button, label, request_id]() {
      const auto pending = pending_requests_.find(label);
      if (pending == pending_requests_.end() || pending->second != request_id) {
        return;  // answered, or superseded by a newer request
      }
      pending_requests_.erase(pending);
      client->remove_pending_request(request_id);
      button->setEnabled(true);
      appendLog(QString("[panel] %1: no reply, gave up.").arg(label));
    });
}

QPushButton * AutoMapperPanel::buttonFor(const QString & label) const
{
  return label == kScanLabel ? scan_button_ : reset_map_button_;
}

void AutoMapperPanel::onServiceReplied(
  const QString & label, bool success, const QString & message)
{
  pending_requests_.erase(label);
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
