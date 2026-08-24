#pragma once

#include <QString>
#include <QVector>

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

class MotorPlotRenderer final
{
public:
    static bool savePng(const QVector<MotorSample> &samples,
                        MotorControlMode mode,
                        const QString &pidSummary,
                        const QString &filePath,
                        QString *errorMessage = nullptr);
};
