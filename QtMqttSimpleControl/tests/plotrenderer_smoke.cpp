#include "../src/plotrenderer.h"

#include <QApplication>
#include <QFile>
#include <QImage>
#include <QTemporaryDir>

#include <cmath>

int main(int argc, char *argv[])
{
    QApplication application(argc, argv);
    QTemporaryDir directory;
    if (!directory.isValid()) {
        return 1;
    }

    QVector<MotorSample> samples;
    for (int index = 0; index <= 28; ++index) {
        const double time = index * 0.25;
        MotorSample sample;
        sample.timeSeconds = time;
        sample.iqReferenceMa = 1000.0;
        sample.iqMa = 1000.0 * (1.0 - std::exp(-time));
        sample.idReferenceMa = 0.0;
        sample.idMa = 35.0 * std::sin(time * 4.0);
        sample.referenceSpeedRpm = 500.0;
        sample.measuredSpeedRpm = 500.0 * (1.0 - std::exp(-time * 0.8));
        sample.targetPositionDeg = 90.0;
        sample.currentPositionDeg = 90.0 * (1.0 - std::exp(-time));
        samples.append(sample);
    }

    const QString path = directory.filePath(QStringLiteral("smoke.png"));
    QString error;
    if (!MotorPlotRenderer::savePng(
            samples, MotorControlMode::Speed,
            QStringLiteral("Speed[Kp=2144, Ki=5, Kd=0]"),
            path, &error)) {
        return 2;
    }
    const QImage result(path);
    if (result.isNull() || result.size() != QSize(1800, 2000)) {
        return 3;
    }

    const QString csvPath = directory.filePath(QStringLiteral("smoke.csv"));
    if (!MotorPlotRenderer::saveCsv(samples, csvPath, &error)) {
        return 4;
    }
    QFile csv(csvPath);
    if (!csv.open(QIODevice::ReadOnly | QIODevice::Text) ||
        !csv.readLine().startsWith("time_s,speed_rpm,speed_ref_rpm")) {
        return 5;
    }
    csv.seek(0);
    if (csv.readAll().count('\n') != samples.size() + 1) {
        return 9;
    }

    const SpeedTestMetrics metrics =
        MotorPlotRenderer::calculateSpeedMetrics(samples);
    if (metrics.sampleCount != samples.size() ||
        std::abs(metrics.targetRpm - 500.0) > 0.01 ||
        !std::isfinite(metrics.riseTimeSeconds) ||
        !std::isfinite(metrics.samplePeriodMs)) {
        return 6;
    }

    QVector<MotorSample> negativeSamples;
    for (int index = 0; index <= 700; ++index) {
        const double time = index * 0.01;
        MotorSample sample;
        sample.timeSeconds = time;
        sample.referenceSpeedRpm = -500.0;
        sample.measuredSpeedRpm = -500.0 * (1.0 - std::exp(-time * 8.0));
        negativeSamples.append(sample);
    }
    const SpeedTestMetrics negativeMetrics =
        MotorPlotRenderer::calculateSpeedMetrics(negativeSamples);
    if (std::abs(negativeMetrics.targetRpm + 500.0) > 0.01 ||
        !std::isfinite(negativeMetrics.riseTimeSeconds) ||
        negativeMetrics.overshootPercent < 0.0) {
        return 7;
    }
    if (std::abs(MotorPlotRenderer::shortestAngleErrorDeg(1.0, 359.0)
                 - 2.0) > 0.001) {
        return 8;
    }
    return 0;
}
