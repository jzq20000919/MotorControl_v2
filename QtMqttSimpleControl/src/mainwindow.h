#pragma once

#include "mqttclient.h"
#include "plotwidget.h"

#include <QElapsedTimer>
#include <QMainWindow>
#include <QVector>

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

private slots:
    void toggleConnection();
    void sendSpeed();
    void sendPosition();
    void startMotor();
    void stopMotor();
    void setConnected(bool connected, const QString &message);
    void processTelemetry(const QString &topic, const QByteArray &payload);

private:
    QByteArray makeCommand(const QString &command, qint64 value);
    bool publishCommand(const QString &command, qint64 value,
                        const QString &description);
    void showMqttStatus(const QString &state, const QString &color,
                        const QString &message);
    void beginRecording(MotorControlMode mode);
    void finishRecording(const QString &reason);

    MqttClient mqtt_;
    QLineEdit *hostEdit_ = nullptr;
    QSpinBox *portSpin_ = nullptr;
    QPushButton *connectButton_ = nullptr;
    QLabel *mqttIndicator_ = nullptr;
    QLabel *mqttStateLabel_ = nullptr;
    QLabel *statusLabel_ = nullptr;
    QSlider *speedSlider_ = nullptr;
    QLabel *speedValueLabel_ = nullptr;
    QPushButton *sendSpeedButton_ = nullptr;
    QSlider *positionSlider_ = nullptr;
    QLabel *positionValueLabel_ = nullptr;
    QPushButton *sendPositionButton_ = nullptr;
    QPushButton *startButton_ = nullptr;
    QPushButton *stopButton_ = nullptr;
    QLabel *recordStateLabel_ = nullptr;
    QLabel *telemetryLabel_ = nullptr;
    MotorPlotWidget *plotWidget_ = nullptr;

    QElapsedTimer recordClock_;
    QVector<MotorSample> samples_;
    MotorControlMode recordingMode_ = MotorControlMode::Speed;
    bool recording_ = false;
    bool motorRunning_ = false;
    quint32 nextCommandId_ = 0U;
};
