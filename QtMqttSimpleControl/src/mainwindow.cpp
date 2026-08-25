#include "mainwindow.h"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QStandardPaths>
#include <QStringList>
#include <QUrl>
#include <QVBoxLayout>

namespace {
const QString kCommandTopic = QStringLiteral("motor/control/command");
const QString kTelemetryTopic = QStringLiteral("motor/control/telemetry");
const QString kAckTopic = QStringLiteral("motor/control/ack");
const QString kTestStatusTopic = QStringLiteral("motor/control/test/status");
const QString kTestDataTopic = QStringLiteral("motor/control/test/data");
constexpr int kPidControllerCount = 4;
constexpr int kDefaultPid[kPidControllerCount][3] = {
    {2144, 5, 0},
    {48, 4, 8},
    {3633, 2693, 0},
    {3633, 2693, 0}
};

quint16 readU16Le(const char *data)
{
    return static_cast<quint16>(static_cast<quint8>(data[0])) |
        (static_cast<quint16>(static_cast<quint8>(data[1])) << 8U);
}

qint16 readS16Le(const char *data)
{
    return static_cast<qint16>(readU16Le(data));
}

quint32 readU32Le(const char *data)
{
    return static_cast<quint32>(static_cast<quint8>(data[0])) |
        (static_cast<quint32>(static_cast<quint8>(data[1])) << 8U) |
        (static_cast<quint32>(static_cast<quint8>(data[2])) << 16U) |
        (static_cast<quint32>(static_cast<quint8>(data[3])) << 24U);
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , mqtt_(this)
{
    setWindowTitle(tr("MQTT 电机 PID 测试"));
    resize(920, 720);
    setMinimumSize(820, 650);

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);

    auto *brokerGroup = new QGroupBox(tr("Broker"), central);
    auto *brokerLayout = new QHBoxLayout(brokerGroup);
    hostEdit_ = new QLineEdit(QStringLiteral("192.168.10.7"), brokerGroup);
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

    auto *statusGroup = new QGroupBox(tr("连接与遥测状态"), central);
    auto *statusLayout = new QGridLayout(statusGroup);
    mqttIndicator_ = new QLabel(QStringLiteral("●"), statusGroup);
    mqttIndicator_->setStyleSheet(
        QStringLiteral("color:#d32f2f;font-size:20px;"));
    mqttStateLabel_ = new QLabel(tr("MQTT OFFLINE"), statusGroup);
    statusLabel_ = new QLabel(tr("未连接"), statusGroup);
    telemetryLabel_ = new QLabel(tr("尚未收到电机遥测"), statusGroup);
    statusLayout->addWidget(mqttIndicator_, 0, 0);
    statusLayout->addWidget(mqttStateLabel_, 0, 1);
    statusLayout->addWidget(statusLabel_, 0, 2);
    statusLayout->addWidget(telemetryLabel_, 1, 0, 1, 3);
    root->addWidget(statusGroup);

    auto *pidGroup = new QGroupBox(tr("PID 参数（MCSDK 增益分子）"), central);
    auto *pidLayout = new QGridLayout(pidGroup);
    pidLayout->addWidget(new QLabel(tr("控制器"), pidGroup), 0, 0);
    pidLayout->addWidget(new QLabel(tr("Kp"), pidGroup), 0, 1);
    pidLayout->addWidget(new QLabel(tr("Ki"), pidGroup), 0, 2);
    pidLayout->addWidget(new QLabel(tr("Kd"), pidGroup), 0, 3);
    pidLayout->addWidget(new QLabel(tr("固定分频"), pidGroup), 0, 4);

    const std::array<QString, 4> controllerNames = {
        tr("速度"), tr("位置"), tr("Iq 电流"), tr("Id 电流")
    };
    const std::array<QString, 4> divisorDescriptions = {
        tr("Kp/2048, Ki/16384；Kd禁用"),
        tr("Kp/1024, Ki/32768, Kd/16"),
        tr("Kp/128, Ki/512；Kd禁用"),
        tr("Kp/128, Ki/512；Kd禁用")
    };
    for (int row = 0; row < kPidControllerCount; ++row) {
        pidLayout->addWidget(new QLabel(controllerNames[row], pidGroup),
                             row + 1, 0);
        QSpinBox *editors[3] = {
            new QSpinBox(pidGroup), new QSpinBox(pidGroup),
            new QSpinBox(pidGroup)
        };
        for (int column = 0; column < 3; ++column) {
            editors[column]->setRange(0, 32767);
            editors[column]->setKeyboardTracking(false);
            pidLayout->addWidget(editors[column], row + 1, column + 1);
        }
        pidEditors_[row] = {editors[0], editors[1], editors[2]};
        if (row != 1) {
            editors[2]->setEnabled(false);
            editors[2]->setToolTip(tr("当前工程的该控制环使用 PI 路径，Kd 未启用"));
        }
        pidLayout->addWidget(new QLabel(divisorDescriptions[row], pidGroup),
                             row + 1, 4);
    }

    auto *pidButtons = new QHBoxLayout();
    applyPidButton_ = new QPushButton(tr("临时应用 PID"), pidGroup);
    savePidButton_ = new QPushButton(tr("保存 PID 参数"), pidGroup);
    restorePidButton_ = new QPushButton(tr("恢复工程默认 PID"), pidGroup);
    pidButtons->addWidget(applyPidButton_);
    pidButtons->addWidget(savePidButton_);
    pidButtons->addWidget(restorePidButton_);
    pidButtons->addStretch();
    pidLayout->addLayout(pidButtons, 5, 0, 1, 5);
    root->addWidget(pidGroup);

    auto *testGroup = new QGroupBox(tr("ESP32 本地 PID 测试接收"), central);
    auto *testForm = new QFormLayout(testGroup);
    auto *localTestHint = new QLabel(
        tr("请在 ESP32 的 PID TEST 页面启动速度或位置测试。Qt 仅接收测试状态、"
           "重组上传数据并保存 PNG 分析报告与同名 CSV。"), testGroup);
    localTestHint->setWordWrap(true);
    testForm->addRow(tr("启动方式"), localTestHint);

    auto *testStatusRow = new QWidget(testGroup);
    auto *testStatusLayout = new QHBoxLayout(testStatusRow);
    testStatusLayout->setContentsMargins(0, 0, 0, 0);
    testStateLabel_ = new QLabel(tr("等待 ESP32 本地测试"), testStatusRow);
    stopButton_ = new QPushButton(tr("立即停止"), testStatusRow);
    stopButton_->setStyleSheet(QStringLiteral(
        "QPushButton{background:#c62828;color:white;padding:7px;}"));
    testStatusLayout->addWidget(testStateLabel_, 1);
    testStatusLayout->addWidget(stopButton_);
    testForm->addRow(tr("测试状态"), testStatusRow);
    root->addWidget(testGroup);

    auto *fileGroup = new QGroupBox(tr("测试报告文件"), central);
    auto *fileLayout = new QGridLayout(fileGroup);
    outputFolderEdit_ = new QLineEdit(defaultOutputFolder(), fileGroup);
    outputFolderEdit_->setReadOnly(true);
    browseOutputButton_ = new QPushButton(tr("选择文件夹"), fileGroup);
    openOutputButton_ = new QPushButton(tr("打开文件夹"), fileGroup);
    lastFileLabel_ = new QLabel(tr("尚未生成测试报告"), fileGroup);
    lastFileLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    lastFileLabel_->setWordWrap(true);
    fileLayout->addWidget(outputFolderEdit_, 0, 0);
    fileLayout->addWidget(browseOutputButton_, 0, 1);
    fileLayout->addWidget(openOutputButton_, 0, 2);
    fileLayout->addWidget(lastFileLabel_, 1, 0, 1, 3);
    root->addWidget(fileGroup);
    root->addStretch();
    setCentralWidget(central);

    connect(connectButton_, &QPushButton::clicked,
            this, &MainWindow::toggleConnection);
    connect(stopButton_, &QPushButton::clicked,
            this, &MainWindow::emergencyStop);
    connect(applyPidButton_, &QPushButton::clicked,
            this, &MainWindow::applyPidParameters);
    connect(savePidButton_, &QPushButton::clicked,
            this, &MainWindow::savePidParameters);
    connect(restorePidButton_, &QPushButton::clicked,
            this, &MainWindow::restoreDefaultPidParameters);
    connect(browseOutputButton_, &QPushButton::clicked,
            this, &MainWindow::chooseOutputFolder);
    connect(openOutputButton_, &QPushButton::clicked,
            this, &MainWindow::openOutputFolder);
    connect(&mqtt_, &MqttClient::connected, this, [this] {
        if (!mqtt_.subscribeQos1(kTelemetryTopic) ||
            !mqtt_.subscribeQos1(kAckTopic) ||
            !mqtt_.subscribeQos1(kTestStatusTopic) ||
            !mqtt_.subscribeQos1(kTestDataTopic)) {
            mqtt_.disconnectFromBroker();
            setConnected(false, tr("电机主题订阅失败"));
            return;
        }
        setConnected(true, tr("已连接，等待电机遥测"));
    });
    connect(&mqtt_, &MqttClient::disconnected, this, [this] {
        setConnected(false, tr("MQTT Broker 已断开"));
    });
    connect(&mqtt_, &MqttClient::messageReceived,
            this, &MainWindow::processMqttMessage);
    connect(&mqtt_, &MqttClient::errorOccurred, this,
            [this](const QString &message) {
                setConnected(false, tr("连接错误：%1").arg(message));
                showMqttStatus(QStringLiteral("MQTT ERROR"),
                               QStringLiteral("#d32f2f"),
                               tr("连接错误：%1").arg(message));
            });

    loadPidParameters();
    setConnected(false, tr("未连接"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if ((testState_ != TestState::Idle || motorRunning_) &&
        mqtt_.isConnected()) {
        (void)mqtt_.publishQos1(
            kCommandTopic, makeCommand(QStringLiteral("stop"), 0));
    }
    event->accept();
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
    const QString clientId = QStringLiteral("qt-pid-test-%1")
        .arg(QCoreApplication::applicationPid());
    mqtt_.connectToBroker(
        host, static_cast<quint16>(portSpin_->value()), clientId);
}

void MainWindow::emergencyStop()
{
    if (mqttConnected_) {
        (void)publishCommand(QStringLiteral("stop"), 0, tr("立即停止命令"));
    }
    if (testState_ != TestState::Idle) {
        testStateLabel_->setText(
            tr("已请求 ESP32 提前停止，等待回传已记录的数据…"));
    }
    updateControlAvailability();
}

void MainWindow::saveTestImage(const QString &resultText)
{
    if (samples_.isEmpty()) {
        lastFileLabel_->setText(tr("没有采样数据，未生成测试报告"));
        return;
    }
    const QString folder = outputFolderEdit_->text();
    if (!QDir().mkpath(folder)) {
        lastFileLabel_->setText(tr("无法创建输出文件夹：%1").arg(folder));
        return;
    }
    const QString modeName = testMode_ == MotorControlMode::Speed
        ? QStringLiteral("speed") : QStringLiteral("position");
    const QString baseName = QStringLiteral("%1_pid_test_%2")
        .arg(modeName,
             QDateTime::currentDateTime().toString(
                 QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    const QString pngPath = QDir(folder).filePath(
        baseName + QStringLiteral(".png"));
    const QString csvPath = QDir(folder).filePath(
        baseName + QStringLiteral(".csv"));
    QString pngError;
    QString csvError;
    const bool pngSaved = MotorPlotRenderer::savePng(
        samples_, testMode_, pidSummary(), pngPath, &pngError);
    const bool csvSaved = MotorPlotRenderer::saveCsv(
        samples_, csvPath, &csvError);
    if (!pngSaved || !csvSaved) {
        QStringList errors;
        if (!pngSaved) {
            errors.append(tr("PNG：%1").arg(pngError));
        }
        if (!csvSaved) {
            errors.append(tr("CSV：%1").arg(csvError));
        }
        lastFileLabel_->setText(
            tr("测试报告保存不完整：%1").arg(errors.join(QStringLiteral("；"))));
        return;
    }
    lastFileLabel_->setText(
        tr("%1：%2；%3（%4 个原始采样点）")
            .arg(resultText,
                 QDir::toNativeSeparators(pngPath),
                 QDir::toNativeSeparators(csvPath))
            .arg(samples_.size()));
}

void MainWindow::processMqttMessage(const QString &topic,
                                    const QByteArray &payload)
{
    if (topic == kTestDataTopic) {
        processTestData(payload);
        return;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(payload, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        statusLabel_->setText(tr("收到无效 JSON：%1")
                              .arg(parseError.errorString()));
        return;
    }
    const QJsonObject object = document.object();

    if (topic == kAckTopic) {
        const quint32 id = static_cast<quint32>(
            object.value(QStringLiteral("id")).toInteger());
        const bool accepted = object.value(QStringLiteral("ok")).toBool();
        const QString message =
            object.value(QStringLiteral("message")).toString();
        statusLabel_->setText(tr("电机网关：%1").arg(message));
        if (id == activeTestId_ && !accepted) {
            testStateLabel_->setText(tr("测试被拒绝：%1").arg(message));
            resetTestState();
        }
        return;
    }
    if (topic == kTestStatusTopic) {
        const quint32 id = static_cast<quint32>(
            object.value(QStringLiteral("id")).toInteger());
        const QString stage =
            object.value(QStringLiteral("stage")).toString();
        const QString message =
            object.value(QStringLiteral("message")).toString();
        const int sampleCount =
            object.value(QStringLiteral("samples")).toInt();
        const int samplePeriodUs =
            object.value(QStringLiteral("sample_period_us")).toInt();

        if (activeTestId_ == 0U &&
            (stage == QStringLiteral("accepted") ||
             stage == QStringLiteral("recording") ||
             stage == QStringLiteral("sending") ||
             stage == QStringLiteral("publishing"))) {
            samples_.clear();
            receivedSamples_.clear();
            expectedSampleCount_ = 0U;
            receivedSampleCount_ = 0U;
            activeTestId_ = id;
            testMode_ = object.value(QStringLiteral("mode")).toInt() == 1
                ? MotorControlMode::Position : MotorControlMode::Speed;
            testState_ = stage == QStringLiteral("sending") ||
                         stage == QStringLiteral("publishing")
                ? TestState::Receiving : TestState::Running;
            updateControlAvailability();
        }
        if (id != activeTestId_ || activeTestId_ == 0U) {
            if (stage == QStringLiteral("error")) {
                testStateLabel_->setText(
                    tr("ESP32 本地测试失败：%1").arg(message));
            }
            return;
        }

        if (stage == QStringLiteral("sending") ||
            stage == QStringLiteral("publishing")) {
            testState_ = TestState::Receiving;
            testStateLabel_->setText(
                tr("测试已停止，ESP32 正在回传 %1 个 CAN 采样点…")
                    .arg(sampleCount));
        } else if (stage == QStringLiteral("complete") ||
                   stage == QStringLiteral("aborted")) {
            finalizeReceivedTest(stage == QStringLiteral("aborted"), message);
        } else if (stage == QStringLiteral("error")) {
            testStateLabel_->setText(tr("ESP32 测试失败：%1").arg(message));
            resetTestState();
        } else if (stage == QStringLiteral("accepted") ||
                   stage == QStringLiteral("recording")) {
            QString detail = tr("ESP32：%1").arg(message);
            if (sampleCount > 0) {
                detail += tr("（%1 点，平均 %2 μs）")
                    .arg(sampleCount)
                    .arg(samplePeriodUs);
            }
            testStateLabel_->setText(detail);
        }
        return;
    }
    if (topic != kTelemetryTopic) {
        return;
    }

    telemetrySeen_ = true;
    linkActive_ = object.value(QStringLiteral("link_active")).toBool();
    const bool running = object.value(QStringLiteral("running")).toBool();
    const int mode = object.value(QStringLiteral("mode")).toInt();
    telemetryLabel_->setText(
        tr("STM32链路 %1 | %2 | %3 | 实验曲线由 ESP32 本地高速缓存")
            .arg(linkActive_ ? tr("在线") : tr("离线"))
            .arg(running ? tr("运行") : tr("停止"))
            .arg(mode == 1 ? tr("位置模式") : tr("速度模式")));
    motorRunning_ = running;
    updateControlAvailability();
}

void MainWindow::processTestData(const QByteArray &payload)
{
    constexpr int headerSize = 20;
    constexpr int recordSize = 20;
    if (payload.size() < headerSize ||
        payload.left(4) != QByteArrayLiteral("MCTD") ||
        static_cast<quint8>(payload[4]) != 1U ||
        static_cast<quint8>(payload[6]) != recordSize) {
        statusLabel_->setText(tr("收到格式无效的测试数据包"));
        return;
    }
    const char *data = payload.constData();
    const MotorControlMode mode = static_cast<quint8>(data[5]) == 1U
        ? MotorControlMode::Position : MotorControlMode::Speed;
    const quint32 id = readU32Le(data + 8);
    const quint32 startIndex = readU32Le(data + 12);
    const quint16 count = readU16Le(data + 16);
    const quint16 total = readU16Le(data + 18);
    if (activeTestId_ == 0U) {
        activeTestId_ = id;
        testMode_ = mode;
        testState_ = TestState::Receiving;
        samples_.clear();
        receivedSamples_.clear();
        expectedSampleCount_ = 0U;
        receivedSampleCount_ = 0U;
        updateControlAvailability();
    }
    if (id != activeTestId_) {
        return;
    }
    if (total == 0U || startIndex + count > total ||
        payload.size() < headerSize + count * recordSize) {
        statusLabel_->setText(tr("测试数据包长度或索引无效"));
        return;
    }
    if (expectedSampleCount_ == 0U) {
        expectedSampleCount_ = total;
        testMode_ = mode;
        samples_.resize(total);
        receivedSamples_.fill(false, total);
    } else if (expectedSampleCount_ != total || testMode_ != mode) {
        statusLabel_->setText(tr("测试数据集标识不一致，已忽略数据包"));
        return;
    }

    for (quint16 index = 0U; index < count; ++index) {
        const char *record = data + headerSize + index * recordSize;
        MotorSample sample;
        sample.timeSeconds = readU32Le(record) / 1000000.0;
        sample.measuredSpeedRpm = readS16Le(record + 4);
        sample.referenceSpeedRpm = readS16Le(record + 6);
        sample.currentPositionDeg = readU16Le(record + 8) / 100.0;
        sample.targetPositionDeg = readU16Le(record + 10) / 100.0;
        sample.iqMa = readS16Le(record + 12);
        sample.idMa = readS16Le(record + 14);
        sample.iqReferenceMa = readS16Le(record + 16);
        sample.idReferenceMa = readS16Le(record + 18);
        const int destination = static_cast<int>(startIndex + index);
        samples_[destination] = sample;
        if (!receivedSamples_[destination]) {
            receivedSamples_[destination] = true;
            ++receivedSampleCount_;
        }
    }
    testState_ = TestState::Receiving;
    testStateLabel_->setText(
        tr("正在接收测试数据：%1 / %2 个 CAN 采样点")
            .arg(receivedSampleCount_)
            .arg(expectedSampleCount_));
}

void MainWindow::finalizeReceivedTest(bool aborted, const QString &message)
{
    if (expectedSampleCount_ == 0U ||
        receivedSampleCount_ != expectedSampleCount_) {
        testStateLabel_->setText(
            tr("数据回传不完整：收到 %1 / %2 点；%3")
                .arg(receivedSampleCount_)
                .arg(expectedSampleCount_)
                .arg(message));
    } else {
        const QString result = aborted
            ? tr("测试提前停止，部分曲线已保存")
            : tr("7 秒 CAN 高速采样测试完成");
        saveTestImage(result);
        const double averageUs = samples_.size() > 1
            ? samples_.last().timeSeconds * 1000000.0 /
                  (samples_.size() - 1)
            : 0.0;
        testStateLabel_->setText(
            tr("%1；共 %2 点，平均采样周期 %3 μs")
                .arg(result)
                .arg(samples_.size())
                .arg(averageUs, 0, 'f', 1));
    }
    resetTestState();
}

void MainWindow::resetTestState()
{
    testState_ = TestState::Idle;
    activeTestId_ = 0U;
    expectedSampleCount_ = 0U;
    receivedSampleCount_ = 0U;
    samples_.clear();
    receivedSamples_.clear();
    updateControlAvailability();
}

void MainWindow::applyPidParameters()
{
    if (motorRunning_) {
        statusLabel_->setText(tr("请先停止电机，再应用 PID 参数"));
        return;
    }
    if (sendAllPidParameters()) {
        statusLabel_->setText(
            tr("当前 PID 已发送至 ESP32 临时缓存；不会改写 STM32 固件默认值"));
    }
}

bool MainWindow::sendAllPidParameters()
{
    if (!mqttConnected_) {
        statusLabel_->setText(tr("MQTT 未连接，无法下发 PID"));
        return false;
    }
    if (motorRunning_) {
        statusLabel_->setText(tr("电机运行中禁止修改 PID"));
        return false;
    }
    const auto values = pidEditorValues();
    for (int controller = 0; controller < kPidControllerCount; ++controller) {
        if (!publishPidCommand(controller, values[controller])) {
            return false;
        }
    }
    return true;
}

bool MainWindow::publishPidCommand(int controller, const PidValues &values)
{
    QJsonObject object;
    object.insert(QStringLiteral("id"),
                  static_cast<qint64>(++nextCommandId_));
    object.insert(QStringLiteral("cmd"), QStringLiteral("set_pid"));
    object.insert(QStringLiteral("controller"), controller);
    object.insert(QStringLiteral("kp"), values.kp);
    object.insert(QStringLiteral("ki"), values.ki);
    if (controller == 1) {
        object.insert(QStringLiteral("kd"), values.kd);
    }
    return mqtt_.publishQos1(
        kCommandTopic, QJsonDocument(object).toJson(QJsonDocument::Compact));
}

void MainWindow::savePidParameters()
{
    QSettings settings(QStringLiteral("MotorControl_v2"),
                       QStringLiteral("QtMqttSimpleControl"));
    const auto values = pidEditorValues();
    settings.beginGroup(QStringLiteral("pid"));
    settings.setValue(QStringLiteral("version"), 1);
    const std::array<QString, 4> names = {
        QStringLiteral("speed"), QStringLiteral("position"),
        QStringLiteral("iq"), QStringLiteral("id")
    };
    for (int index = 0; index < kPidControllerCount; ++index) {
        settings.setValue(names[index] + QStringLiteral("/kp"), values[index].kp);
        settings.setValue(names[index] + QStringLiteral("/ki"), values[index].ki);
        settings.setValue(names[index] + QStringLiteral("/kd"), values[index].kd);
    }
    settings.endGroup();
    settings.sync();
    statusLabel_->setText(
        settings.status() == QSettings::NoError
            ? tr("PID 参数已保存；下次启动将自动载入")
            : tr("PID 参数保存失败"));
}

void MainWindow::restoreDefaultPidParameters()
{
    std::array<PidValues, 4> defaults;
    for (int index = 0; index < kPidControllerCount; ++index) {
        defaults[index] = {
            kDefaultPid[index][0], kDefaultPid[index][1],
            kDefaultPid[index][2]
        };
    }
    setPidEditors(defaults);
    QSettings settings(QStringLiteral("MotorControl_v2"),
                       QStringLiteral("QtMqttSimpleControl"));
    settings.remove(QStringLiteral("pid"));
    settings.sync();
    if (mqttConnected_ && !motorRunning_) {
        (void)sendAllPidParameters();
        statusLabel_->setText(tr("已恢复并缓存当前工程默认 PID"));
    } else {
        statusLabel_->setText(tr("已恢复工程默认 PID；MQTT 连接后可临时应用"));
    }
}

void MainWindow::loadPidParameters()
{
    std::array<PidValues, 4> values;
    for (int index = 0; index < kPidControllerCount; ++index) {
        values[index] = {
            kDefaultPid[index][0], kDefaultPid[index][1],
            kDefaultPid[index][2]
        };
    }
    QSettings settings(QStringLiteral("MotorControl_v2"),
                       QStringLiteral("QtMqttSimpleControl"));
    settings.beginGroup(QStringLiteral("pid"));
    if (settings.value(QStringLiteral("version"), 0).toInt() == 1) {
        const std::array<QString, 4> names = {
            QStringLiteral("speed"), QStringLiteral("position"),
            QStringLiteral("iq"), QStringLiteral("id")
        };
        for (int index = 0; index < kPidControllerCount; ++index) {
            values[index].kp = settings.value(
                names[index] + QStringLiteral("/kp"), values[index].kp).toInt();
            values[index].ki = settings.value(
                names[index] + QStringLiteral("/ki"), values[index].ki).toInt();
            values[index].kd = settings.value(
                names[index] + QStringLiteral("/kd"), values[index].kd).toInt();
        }
    }
    settings.endGroup();
    setPidEditors(values);
}

void MainWindow::setPidEditors(const std::array<PidValues, 4> &values)
{
    for (int index = 0; index < kPidControllerCount; ++index) {
        pidEditors_[index].kp->setValue(values[index].kp);
        pidEditors_[index].ki->setValue(values[index].ki);
        pidEditors_[index].kd->setValue(index == 1 ? values[index].kd : 0);
    }
}

std::array<MainWindow::PidValues, 4> MainWindow::pidEditorValues() const
{
    std::array<PidValues, 4> values;
    for (int index = 0; index < kPidControllerCount; ++index) {
        values[index] = {
            pidEditors_[index].kp->value(),
            pidEditors_[index].ki->value(),
            pidEditors_[index].kd->value()
        };
    }
    return values;
}

QString MainWindow::pidSummary() const
{
    const auto values = pidEditorValues();
    const std::array<QString, 4> names = {
        tr("速度"), tr("位置"), tr("Iq"), tr("Id")
    };
    QStringList parts;
    for (int index = 0; index < kPidControllerCount; ++index) {
        parts.append(tr("%1[Kp=%2, Ki=%3, Kd=%4]")
                         .arg(names[index])
                         .arg(values[index].kp)
                         .arg(values[index].ki)
                         .arg(values[index].kd));
    }
    return parts.join(QStringLiteral("   "));
}

void MainWindow::chooseOutputFolder()
{
    const QString folder = QFileDialog::getExistingDirectory(
        this, tr("选择测试报告保存文件夹"), outputFolderEdit_->text());
    if (!folder.isEmpty()) {
        outputFolderEdit_->setText(QDir::toNativeSeparators(folder));
    }
}

void MainWindow::openOutputFolder()
{
    QDir().mkpath(outputFolderEdit_->text());
    QDesktopServices::openUrl(
        QUrl::fromLocalFile(outputFolderEdit_->text()));
}

QString MainWindow::defaultOutputFolder() const
{
    QString documents =
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    if (documents.isEmpty()) {
        documents = QCoreApplication::applicationDirPath();
    }
    return QDir::toNativeSeparators(
        QDir(documents).filePath(QStringLiteral("MotorControl_PID_Tests")));
}

void MainWindow::setConnected(bool connected, const QString &message)
{
    mqttConnected_ = connected;
    if (!connected) {
        telemetrySeen_ = false;
        linkActive_ = false;
        motorRunning_ = false;
        if (testState_ != TestState::Idle) {
            testStateLabel_->setText(
                tr("MQTT 已断开，本次测试数据可能丢失，请重新测试"));
            resetTestState();
        }
    }
    hostEdit_->setEnabled(!connected);
    portSpin_->setEnabled(!connected);
    connectButton_->setText(connected ? tr("断开") : tr("连接"));
    showMqttStatus(
        connected ? QStringLiteral("MQTT ONLINE")
                  : QStringLiteral("MQTT OFFLINE"),
        connected ? QStringLiteral("#2e7d32")
                  : QStringLiteral("#d32f2f"),
        message);
    updateControlAvailability();
}

void MainWindow::updateControlAvailability()
{
    const bool idle = testState_ == TestState::Idle;
    const bool ready = mqttConnected_ && !motorRunning_ && idle;
    applyPidButton_->setEnabled(ready);
    savePidButton_->setEnabled(idle);
    restorePidButton_->setEnabled(idle);
    for (int index = 0; index < kPidControllerCount; ++index) {
        pidEditors_[index].kp->setEnabled(idle);
        pidEditors_[index].ki->setEnabled(idle);
        pidEditors_[index].kd->setEnabled(idle && index == 1);
    }
    stopButton_->setEnabled(mqttConnected_);
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

QByteArray MainWindow::makeCommand(const QString &command, qint64 value)
{
    QJsonObject object;
    object.insert(QStringLiteral("id"),
                  static_cast<qint64>(++nextCommandId_));
    object.insert(QStringLiteral("cmd"), command);
    object.insert(QStringLiteral("value"), value);
    return QJsonDocument(object).toJson(QJsonDocument::Compact);
}

void MainWindow::showMqttStatus(const QString &state, const QString &color,
                                const QString &message)
{
    mqttIndicator_->setStyleSheet(
        QStringLiteral("color:%1;font-size:20px;").arg(color));
    mqttStateLabel_->setText(state);
    statusLabel_->setText(message);
}
