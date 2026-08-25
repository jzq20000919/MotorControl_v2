#pragma once

#include <QString>
#include <QVector>

#include <limits>

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

struct SpeedTestMetrics
{
    double targetRpm = std::numeric_limits<double>::quiet_NaN();
    double riseTimeSeconds = std::numeric_limits<double>::quiet_NaN();
    double overshootPercent = std::numeric_limits<double>::quiet_NaN();
    double settlingTimeSeconds = std::numeric_limits<double>::quiet_NaN();
    double steadyStateErrorRpm = std::numeric_limits<double>::quiet_NaN();
    double rmseRpm = std::numeric_limits<double>::quiet_NaN();
    double maximumErrorRpm = std::numeric_limits<double>::quiet_NaN();
    double samplePeriodMs = std::numeric_limits<double>::quiet_NaN();
    int sampleCount = 0;
};

struct PositionTestMetrics
{
    double initialPositionDeg = std::numeric_limits<double>::quiet_NaN();
    double targetPositionDeg = std::numeric_limits<double>::quiet_NaN();
    double riseTimeSeconds = std::numeric_limits<double>::quiet_NaN();
    double overshootPercent = std::numeric_limits<double>::quiet_NaN();
    double settlingTimeSeconds = std::numeric_limits<double>::quiet_NaN();
    double steadyStateErrorDeg = std::numeric_limits<double>::quiet_NaN();
    double rmseDeg = std::numeric_limits<double>::quiet_NaN();
    double maximumErrorDeg = std::numeric_limits<double>::quiet_NaN();
    double samplePeriodMs = std::numeric_limits<double>::quiet_NaN();
    int sampleCount = 0;
};

class MotorPlotRenderer final
{
public:
    static SpeedTestMetrics calculateSpeedMetrics(
        const QVector<MotorSample> &samples);
    static PositionTestMetrics calculatePositionMetrics(
        const QVector<MotorSample> &samples);

    /** 最短有符号角度误差 target-current，结果位于 [-180, 180)。 */
    static double shortestAngleErrorDeg(double targetDeg,
                                        double currentDeg);

    static bool savePng(const QVector<MotorSample> &samples,
                        MotorControlMode mode,
                        const QString &pidSummary,
                        const QString &filePath,
                        QString *errorMessage = nullptr);

    static bool saveCsv(const QVector<MotorSample> &samples,
                        const QString &filePath,
                        QString *errorMessage = nullptr);
};
