#pragma once

#include "mqttclient.h"
#include "plotrenderer.h"

#include <QMainWindow>
#include <QTimer>
#include <QVector>

#include <array>

class QCloseEvent;
class QJsonObject;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;

class MainWindow final : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void toggleConnection();
    void startSpeedTest();
    void startPositionTest();
    void emergencyStop();
    void applyPidParameters();
    void savePidParameters();
    void restoreDefaultPidParameters();
    void chooseOutputFolder();
    void openOutputFolder();
    void processMqttMessage(const QString &topic, const QByteArray &payload);

private:
    enum class TestState
    {
        Idle,
        Running,
        Receiving
    };

    struct PidValues
    {
        int kp = 0;
        int ki = 0;
        int kd = 0;
    };

    struct PidEditor
    {
        QSpinBox *kp = nullptr;
        QSpinBox *ki = nullptr;
        QSpinBox *kd = nullptr;
    };

    QByteArray makeCommand(const QString &command, qint64 value);
    QByteArray makeTestCommand(MotorControlMode mode, quint32 commandId) const;
    bool publishCommand(const QString &command, qint64 value,
                        const QString &description);
    bool publishPidCommand(int controller, const PidValues &values);
    bool sendAllPidParameters();
    void showMqttStatus(const QString &state, const QString &color,
                        const QString &message);
    void setConnected(bool connected, const QString &message);
    void startTest(MotorControlMode mode);
    void saveTestImage(const QString &resultText);
    void processTestData(const QByteArray &payload);
    void finalizeReceivedTest(bool aborted, const QString &message);
    void updateControlAvailability();
    void setPidEditors(const std::array<PidValues, 4> &values);
    std::array<PidValues, 4> pidEditorValues() const;
    void loadPidParameters();
    QString pidSummary() const;
    QString defaultOutputFolder() const;

    MqttClient mqtt_;
    QLineEdit *hostEdit_ = nullptr;
    QSpinBox *portSpin_ = nullptr;
    QPushButton *connectButton_ = nullptr;
    QLabel *mqttIndicator_ = nullptr;
    QLabel *mqttStateLabel_ = nullptr;
    QLabel *statusLabel_ = nullptr;
    std::array<PidEditor, 4> pidEditors_;
    QPushButton *applyPidButton_ = nullptr;
    QPushButton *savePidButton_ = nullptr;
    QPushButton *restorePidButton_ = nullptr;
    QSlider *speedSlider_ = nullptr;
    QLabel *speedValueLabel_ = nullptr;
    QPushButton *speedTestButton_ = nullptr;
    QSlider *positionSlider_ = nullptr;
    QLabel *positionValueLabel_ = nullptr;
    QPushButton *positionTestButton_ = nullptr;
    QPushButton *stopButton_ = nullptr;
    QLabel *testStateLabel_ = nullptr;
    QLabel *telemetryLabel_ = nullptr;
    QLineEdit *outputFolderEdit_ = nullptr;
    QPushButton *browseOutputButton_ = nullptr;
    QPushButton *openOutputButton_ = nullptr;
    QLabel *lastFileLabel_ = nullptr;

    QVector<MotorSample> samples_;
    QVector<bool> receivedSamples_;
    QTimer testCommandTimeoutTimer_;
    TestState testState_ = TestState::Idle;
    MotorControlMode testMode_ = MotorControlMode::Speed;
    quint32 activeTestId_ = 0U;
    quint16 expectedSampleCount_ = 0U;
    quint16 receivedSampleCount_ = 0U;
    bool mqttConnected_ = false;
    bool telemetrySeen_ = false;
    bool linkActive_ = false;
    bool motorRunning_ = false;
    quint32 nextCommandId_ = 0U;
};
