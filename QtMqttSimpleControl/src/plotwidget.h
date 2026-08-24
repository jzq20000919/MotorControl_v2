#pragma once

#include <QVector>
#include <QWidget>

enum class MotorControlMode
{
    Speed = 0,
    Position = 1
};

struct MotorSample
{
    double timeSeconds = 0.0;
    double iqMa = 0.0;
    double idMa = 0.0;
    double iqReferenceMa = 0.0;
    double idReferenceMa = 0.0;
    double measuredSpeedRpm = 0.0;
    double referenceSpeedRpm = 0.0;
    double currentPositionDeg = 0.0;
    double targetPositionDeg = 0.0;
};

class MotorPlotWidget final : public QWidget
{
    Q_OBJECT

public:
    explicit MotorPlotWidget(QWidget *parent = nullptr);

    void clearData();
    void setData(const QVector<MotorSample> &samples, MotorControlMode mode);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    struct PlotSeries
    {
        QString name;
        QColor color;
        double MotorSample::*value = nullptr;
    };

    void drawPanel(QPainter &painter, const QRectF &bounds,
                   const QString &title, const QString &verticalUnit,
                   const QVector<PlotSeries> &series) const;

    QVector<MotorSample> samples_;
    MotorControlMode mode_ = MotorControlMode::Speed;
};
