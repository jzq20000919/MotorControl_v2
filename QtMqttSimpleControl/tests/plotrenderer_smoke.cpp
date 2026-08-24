#include "../src/plotrenderer.h"

#include <QApplication>
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
    return (!result.isNull() && result.size() == QSize(1800, 1120)) ? 0 : 3;
}
