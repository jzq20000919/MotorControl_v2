#include "mainwindow.h"

#include <QCoreApplication>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

namespace {
const QString kCommandTopic = QStringLiteral("motor/control/command");
const QString kTelemetryTopic = QStringLiteral("motor/control/telemetry");
constexpr qsizetype kMaximumSamples = 100000;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , mqtt_(this)
{
    setWindowTitle(tr("MQTT 电机控制与运行曲线"));
    resize(1100, 850);
    setMinimumSize(900, 720);

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);

    auto *brokerGroup = new QGroupBox(tr("Broker"), central);
    auto *brokerLayout = new QHBoxLayout(brokerGroup);
    hostEdit_ = new QLineEdit(QStringLiteral("192.168.10.4"), brokerGroup);
    portSpin_ = new QSpinBox(brokerGroup);
    portSpin_->setRange(1, 65535);
    portSpin_->setValue(1883);
    connectButton_ = new QPushButton(tr("连接"), brokerGroup);
    brokerLayout->addWidget(new QLabel(tr("IP"), brokerGroup));
    brokerLayout->addWidget(hostEdit_, 1);
    brokerLayout->addWidget(new QLabel(tr("端口"), brokerGroup));
    brokerLayout->addWidget(portSpin_);
    brokerLayout->addWidget(connectButton_);
    root->addWidget(brokerGroup);

    auto *statusGroup = new QGroupBox(tr("MQTT 状态"), central);
    auto *statusLayout = new QHBoxLayout(statusGroup);
    mqttIndicator_ = new QLabel(QStringLiteral("●"), statusGroup);
    mqttIndicator_->setStyleSheet(
        QStringLiteral("color:#d32f2f;font-size:20px;"));
    mqttStateLabel_ = new QLabel(tr("MQTT OFFLINE"), statusGroup);
    mqttStateLabel_->setMinimumWidth(135);
    statusLabel_ = new QLabel(tr("未连接"), statusGroup);
    statusLayout->addWidget(mqttIndicator_);
    statusLayout->addWidget(mqttStateLabel_);
    statusLayout->addWidget(statusLabel_, 1);
    root->addWidget(statusGroup);

    auto *commandGroup = new QGroupBox(tr("电机控制"), central);
    auto *form = new QFormLayout(commandGroup);

    auto *speedRow = new QWidget(commandGroup);
    auto *speedLayout = new QHBoxLayout(speedRow);
    speedLayout->setContentsMargins(0, 0, 0, 0);
    speedSlider_ = new QSlider(Qt::Horizontal, speedRow);
    speedSlider_->setRange(-2600, 2600);
    speedSlider_->setSingleStep(100);
    speedSlider_->setPageStep(500);
    speedValueLabel_ = new QLabel(tr("0 RPM"), speedRow);
    speedValueLabel_->setMinimumWidth(82);
    speedValueLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    sendSpeedButton_ = new QPushButton(tr("发送速度"), speedRow);
    speedLayout->addWidget(speedSlider_, 1);
    speedLayout->addWidget(speedValueLabel_);
    speedLayout->addWidget(sendSpeedButton_);
    form->addRow(tr("速度设定"), speedRow);

    auto *positionRow = new QWidget(commandGroup);
    auto *positionLayout = new QHBoxLayout(positionRow);
    positionLayout->setContentsMargins(0, 0, 0, 0);
    positionSlider_ = new QSlider(Qt::Horizontal, positionRow);
    positionSlider_->setRange(0, 35999);
    positionSlider_->setSingleStep(100);
    positionSlider_->setPageStep(1000);
    positionValueLabel_ = new QLabel(tr("0.00°"), positionRow);
    positionValueLabel_->setMinimumWidth(82);
    positionValueLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    sendPositionButton_ = new QPushButton(tr("发送位置"), positionRow);
    positionLayout->addWidget(positionSlider_, 1);
    positionLayout->addWidget(positionValueLabel_);
    positionLayout->addWidget(sendPositionButton_);
    form->addRow(tr("位置设定"), positionRow);

    auto *runRow = new QWidget(commandGroup);
    auto *runLayout = new QHBoxLayout(runRow);
    runLayout->setContentsMargins(0, 0, 0, 0);
    startButton_ = new QPushButton(tr("启动电机"), runRow);
    stopButton_ = new QPushButton(tr("停止电机"), runRow);
    startButton_->setStyleSheet(QStringLiteral(
        "QPushButton{background:#2e7d32;color:white;padding:7px;}"));
    stopButton_->setStyleSheet(QStringLiteral(
        "QPushButton{background:#c62828;color:white;padding:7px;}"));
    runLayout->addWidget(startButton_);
    runLayout->addWidget(stopButton_);
    form->addRow(tr("电机启停"), runRow);
    root->addWidget(commandGroup);

    auto *recordGroup = new QGroupBox(tr("运行数据记录"), central);
    auto *recordLayout = new QVBoxLayout(recordGroup);
    auto *recordStatusLayout = new QHBoxLayout();
    recordStateLabel_ = new QLabel(tr("等待电机启动"), recordGroup);
    telemetryLabel_ = new QLabel(tr("尚未收到遥测"), recordGroup);
    telemetryLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    recordStatusLayout->addWidget(recordStateLabel_, 1);
    recordStatusLayout->addWidget(telemetryLabel_, 2);
    recordLayout->addLayout(recordStatusLayout);
    plotWidget_ = new MotorPlotWidget(recordGroup);
    recordLayout->addWidget(plotWidget_, 1);
    root->addWidget(recordGroup, 1);
    setCentralWidget(central);

    connect(connectButton_, &QPushButton::clicked,
            this, &MainWindow::toggleConnection);
    connect(sendSpeedButton_, &QPushButton::clicked,
            this, &MainWindow::sendSpeed);
    connect(sendPositionButton_, &QPushButton::clicked,
            this, &MainWindow::sendPosition);
    connect(startButton_, &QPushButton::clicked,
            this, &MainWindow::startMotor);
    connect(stopButton_, &QPushButton::clicked,
            this, &MainWindow::stopMotor);
    connect(speedSlider_, &QSlider::valueChanged, this, [this](int value) {
        speedValueLabel_->setText(tr("%1 RPM").arg(value));
    });
    connect(positionSlider_, &QSlider::valueChanged, this, [this](int value) {
        positionValueLabel_->setText(
            tr("%1°").arg(value / 100.0, 0, 'f', 2));
    });
    connect(&mqtt_, &MqttClient::connected, this, [this] {
        if (!mqtt_.subscribeQos1(kTelemetryTopic)) {
            setConnected(false, tr("遥测主题订阅失败"));
            return;
        }
        setConnected(true, tr("已连接并订阅电机遥测"));
    });
    connect(&mqtt_, &MqttClient::disconnected, this, [this] {
        setConnected(false, tr("MQTT Broker 已断开"));
    });
    connect(&mqtt_, &MqttClient::messageReceived,
            this, &MainWindow::processTelemetry);
    connect(&mqtt_, &MqttClient::errorOccurred, this,
            [this](const QString &message) {
                setConnected(false, tr("连接错误：%1").arg(message));
                showMqttStatus(QStringLiteral("MQTT ERROR"),
                               QStringLiteral("#d32f2f"),
                               tr("连接错误：%1").arg(message));
            });

    setConnected(false, tr("未连接"));
}

void MainWindow::toggleConnection()
{
    if (mqtt_.isConnected()) {
        mqtt_.disconnectFromBroker();
        setConnected(false, tr("已断开"));
        return;
    }

    const QString host = hostEdit_->text().trimmed();
    if (host.isEmpty()) {
        showMqttStatus(QStringLiteral("MQTT ERROR"),
                       QStringLiteral("#d32f2f"), tr("请输入 Broker IP"));
        return;
    }
    showMqttStatus(QStringLiteral("MQTT CONNECTING"),
                   QStringLiteral("#f9a825"), tr("正在连接…"));
    const QString clientId = QStringLiteral("qt-motor-recorder-%1")
        .arg(QCoreApplication::applicationPid());
    mqtt_.connectToBroker(
        host, static_cast<quint16>(portSpin_->value()), clientId);
}

void MainWindow::sendSpeed()
{
    publishCommand(QStringLiteral("set_speed"), speedSlider_->value(),
                   tr("速度命令"));
}

void MainWindow::sendPosition()
{
    publishCommand(QStringLiteral("set_position"), positionSlider_->value(),
                   tr("位置命令"));
}

void MainWindow::startMotor()
{
    if (publishCommand(QStringLiteral("start"), 0, tr("启动命令"))) {
        recordStateLabel_->setText(tr("启动命令已发送，等待运行状态确认…"));
    }
}

void MainWindow::stopMotor()
{
    if (publishCommand(QStringLiteral("stop"), 0, tr("停止命令"))) {
        recordStateLabel_->setText(tr("停止命令已发送，等待停止状态确认…"));
    }
}

void MainWindow::setConnected(bool connected, const QString &message)
{
    if (!connected && recording_) {
        finishRecording(tr("MQTT 连接中断，记录提前结束"));
    }
    if (!connected) {
        motorRunning_ = false;
    }
    hostEdit_->setEnabled(!connected);
    portSpin_->setEnabled(!connected);
    connectButton_->setText(connected ? tr("断开") : tr("连接"));
    sendSpeedButton_->setEnabled(connected);
    sendPositionButton_->setEnabled(connected);
    startButton_->setEnabled(connected);
    stopButton_->setEnabled(connected);
    showMqttStatus(
        connected ? QStringLiteral("MQTT ONLINE")
                  : QStringLiteral("MQTT OFFLINE"),
        connected ? QStringLiteral("#2e7d32")
                  : QStringLiteral("#d32f2f"),
        message);
}

void MainWindow::processTelemetry(const QString &topic,
                                  const QByteArray &payload)
{
    if (topic != kTelemetryTopic) {
        return;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(payload, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        statusLabel_->setText(tr("收到无效遥测 JSON：%1")
                              .arg(parseError.errorString()));
        return;
    }

    const QJsonObject object = document.object();
    const bool running = object.value(QStringLiteral("running")).toBool();
    const MotorControlMode mode =
        object.value(QStringLiteral("mode")).toInt() == 1
            ? MotorControlMode::Position
            : MotorControlMode::Speed;

    telemetryLabel_->setText(
        tr("%1 | %2 | Iq %3 mA | Id %4 mA")
            .arg(running ? tr("运行") : tr("停止"))
            .arg(mode == MotorControlMode::Speed
                     ? tr("速度模式") : tr("位置模式"))
            .arg(object.value(QStringLiteral("iq_ma")).toInt())
            .arg(object.value(QStringLiteral("id_ma")).toInt()));

    if (running && !motorRunning_ && !recording_) {
        beginRecording(mode);
    }

    if (running && recording_) {
        /* If the operator changes mode while running, the final secondary
         * panel follows the mode reported by the motor at the end of the run. */
        recordingMode_ = mode;
        MotorSample sample;
        sample.timeSeconds = recordClock_.nsecsElapsed() / 1000000000.0;
        sample.iqMa = object.value(QStringLiteral("iq_ma")).toDouble();
        sample.idMa = object.value(QStringLiteral("id_ma")).toDouble();
        sample.iqReferenceMa =
            object.value(QStringLiteral("iq_ref_ma")).toDouble();
        sample.idReferenceMa =
            object.value(QStringLiteral("id_ref_ma")).toDouble();
        sample.measuredSpeedRpm =
            object.value(QStringLiteral("speed_rpm")).toDouble();
        sample.referenceSpeedRpm =
            object.value(QStringLiteral("speed_ref_rpm")).toDouble();
        sample.currentPositionDeg =
            object.value(QStringLiteral("position_cdeg")).toDouble() / 100.0;
        sample.targetPositionDeg =
            object.value(QStringLiteral("target_cdeg")).toDouble() / 100.0;
        samples_.append(sample);
        recordStateLabel_->setText(
            tr("正在记录：%1 个采样点，%2 s")
                .arg(samples_.size())
                .arg(sample.timeSeconds, 0, 'f', 1));

        if (samples_.size() >= kMaximumSamples) {
            finishRecording(tr("已达到 100000 点记录上限"));
        }
    }

    if (!running && recording_) {
        finishRecording(tr("电机已停止，曲线绘制完成"));
    }
    motorRunning_ = running;
}

bool MainWindow::publishCommand(const QString &command, qint64 value,
                                const QString &description)
{
    const QByteArray payload = makeCommand(command, value);
    if (!mqtt_.publishQos1(kCommandTopic, payload)) {
        statusLabel_->setText(tr("%1发送失败").arg(description));
        return false;
    }
    statusLabel_->setText(tr("%1已发送").arg(description));
    return true;
}

void MainWindow::showMqttStatus(const QString &state, const QString &color,
                                const QString &message)
{
    mqttIndicator_->setStyleSheet(
        QStringLiteral("color:%1;font-size:20px;").arg(color));
    mqttStateLabel_->setText(state);
    statusLabel_->setText(message);
}

void MainWindow::beginRecording(MotorControlMode mode)
{
    samples_.clear();
    samples_.reserve(4096);
    recordingMode_ = mode;
    recording_ = true;
    recordClock_.restart();
    plotWidget_->clearData();
    recordStateLabel_->setText(
        mode == MotorControlMode::Speed
            ? tr("电机已启动：正在记录速度模式数据")
            : tr("电机已启动：正在记录位置模式数据"));
}

void MainWindow::finishRecording(const QString &reason)
{
    if (!recording_) {
        return;
    }
    recording_ = false;
    plotWidget_->setData(samples_, recordingMode_);
    recordStateLabel_->setText(
        tr("%1（%2 个采样点）").arg(reason).arg(samples_.size()));
}

QByteArray MainWindow::makeCommand(const QString &command, qint64 value)
{
    QJsonObject object;
    object.insert(QStringLiteral("id"),
                  static_cast<qint64>(++nextCommandId_));
    object.insert(QStringLiteral("cmd"), command);
    object.insert(QStringLiteral("value"), value);
    return QJsonDocument(object).toJson(QJsonDocument::Compact);
}
