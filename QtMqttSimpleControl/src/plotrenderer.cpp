#include "plotrenderer.h"

#include <QColor>
#include <QCoreApplication>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QRectF>
#include <QSaveFile>
#include <QStringConverter>
#include <QStringList>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <utility>

namespace {
constexpr double kMinimumMagnitude = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr int kReportWidth = 1800;
constexpr int kReportHeight = 2000;

struct PlotSeries
{
    QString name;
    QColor color;
    std::function<double(const MotorSample &)> value;
    Qt::PenStyle penStyle = Qt::SolidLine;
    qreal penWidth = 2.0;
};

struct PlotOptions
{
    double startTimeSeconds = 0.0;
    double endTimeSeconds = 0.0;
    std::optional<double> yRangeStartTimeSeconds;
    std::optional<double> yMinimum;
    std::optional<double> yMaximum;
    bool robustYRange = false;
    bool symmetricYRange = false;
};

QString translated(const char *text)
{
    return QCoreApplication::translate("MotorPlotRenderer", text);
}

PlotSeries memberSeries(const QString &name, const QColor &color,
                        double MotorSample::*member,
                        Qt::PenStyle penStyle, qreal penWidth)
{
    return {name, color,
            [member](const MotorSample &sample) {
                return sample.*member;
            },
            penStyle, penWidth};
}

double percentile(QVector<double> values, double fraction)
{
    values.erase(
        std::remove_if(values.begin(), values.end(), [](double value) {
            return !std::isfinite(value);
        }),
        values.end());
    if (values.isEmpty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    const double bounded = std::clamp(fraction, 0.0, 1.0);
    const double position = bounded * static_cast<double>(values.size() - 1);
    const qsizetype lower = static_cast<qsizetype>(std::floor(position));
    const qsizetype upper = static_cast<qsizetype>(std::ceil(position));
    const double weight = position - static_cast<double>(lower);
    return values[lower] * (1.0 - weight) + values[upper] * weight;
}

double median(QVector<double> values)
{
    return percentile(std::move(values), 0.5);
}

double medianSamplePeriodMs(const QVector<MotorSample> &samples)
{
    QVector<double> intervals;
    intervals.reserve(std::max<qsizetype>(0, samples.size() - 1));
    for (qsizetype index = 1; index < samples.size(); ++index) {
        const double interval =
            samples[index].timeSeconds - samples[index - 1].timeSeconds;
        if (std::isfinite(interval) && interval > 0.0) {
            intervals.append(interval * 1000.0);
        }
    }
    return median(std::move(intervals));
}

template<typename Accessor>
QVector<double> sampleSlice(const QVector<MotorSample> &samples,
                            qsizetype first, qsizetype last,
                            Accessor accessor)
{
    QVector<double> values;
    first = std::clamp<qsizetype>(first, 0, samples.size());
    last = std::clamp<qsizetype>(last, first, samples.size());
    values.reserve(last - first);
    for (qsizetype index = first; index < last; ++index) {
        const double value = accessor(samples[index]);
        if (std::isfinite(value)) {
            values.append(value);
        }
    }
    return values;
}

double circularMeanDeg(const QVector<double> &angles)
{
    double sine = 0.0;
    double cosine = 0.0;
    int count = 0;
    for (double angle : angles) {
        if (!std::isfinite(angle)) {
            continue;
        }
        const double radians = angle * kPi / 180.0;
        sine += std::sin(radians);
        cosine += std::cos(radians);
        ++count;
    }
    if (count == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (std::hypot(sine, cosine) < kMinimumMagnitude) {
        return angles.constLast();
    }
    double result = std::atan2(sine, cosine) * 180.0 / kPi;
    if (result < 0.0) {
        result += 360.0;
    }
    return result;
}

QString formatMetric(double value, int decimals, const QString &suffix)
{
    if (!std::isfinite(value)) {
        return translated("N/A");
    }
    return QStringLiteral("%1%2").arg(value, 0, 'f', decimals).arg(suffix);
}

void normalizeRange(double &minimum, double &maximum, bool robust,
                    bool symmetric)
{
    if (!std::isfinite(minimum) || !std::isfinite(maximum)) {
        minimum = -1.0;
        maximum = 1.0;
        return;
    }
    if (symmetric) {
        const double magnitude = std::max(std::abs(minimum), std::abs(maximum));
        const double padded = std::max(1.0, magnitude * (robust ? 1.10 : 1.08));
        minimum = -padded;
        maximum = padded;
        return;
    }
    if (std::abs(maximum - minimum) < kMinimumMagnitude) {
        const double padding = std::max(1.0, std::abs(maximum) * 0.1);
        minimum -= padding;
        maximum += padding;
        return;
    }
    const double padding = (maximum - minimum) * (robust ? 0.10 : 0.08);
    minimum -= padding;
    maximum += padding;
}

void drawPanel(QPainter &painter, const QRectF &bounds,
               const QVector<MotorSample> &samples,
               const QString &title, const QString &verticalUnit,
               const QVector<PlotSeries> &series,
               const PlotOptions &options)
{
    painter.save();
    painter.setPen(QPen(QColor(195, 195, 195), 1.0));
    painter.setBrush(Qt::white);
    painter.drawRoundedRect(bounds, 5.0, 5.0);

    const QRectF plotArea(bounds.left() + 100.0, bounds.top() + 64.0,
                          bounds.width() - 130.0, bounds.height() - 125.0);
    const double startTime = options.startTimeSeconds;
    double endTime = options.endTimeSeconds;
    if (!(endTime > startTime)) {
        endTime = samples.isEmpty() ? startTime + 1.0
                                    : samples.constLast().timeSeconds;
    }
    if (!(endTime > startTime)) {
        endTime = startTime + 1.0;
    }

    QFont titleFont = painter.font();
    titleFont.setPointSize(14);
    titleFont.setBold(true);
    painter.setFont(titleFont);
    painter.setPen(QColor(35, 35, 35));
    painter.drawText(QRectF(bounds.left() + 18.0, bounds.top() + 10.0,
                            360.0, 30.0),
                     Qt::AlignLeft | Qt::AlignVCenter, title);

    QFont axisFont = painter.font();
    axisFont.setPointSize(9);
    axisFont.setBold(false);
    painter.setFont(axisFont);
    qreal legendX = bounds.left() + 405.0;
    const qreal legendY = bounds.top() + 28.0;
    for (const PlotSeries &item : series) {
        QPen legendPen(item.color);
        legendPen.setStyle(item.penStyle);
        legendPen.setWidthF(item.penWidth);
        painter.setPen(legendPen);
        painter.drawLine(QPointF(legendX, legendY),
                         QPointF(legendX + 30.0, legendY));
        painter.setPen(QColor(55, 55, 55));
        painter.drawText(QPointF(legendX + 38.0, legendY + 5.0), item.name);
        legendX += 54.0 + painter.fontMetrics().horizontalAdvance(item.name);
    }

    const double rangeStartTime = options.yRangeStartTimeSeconds.value_or(
        startTime);
    QVector<double> visibleValues;
    for (const MotorSample &sample : samples) {
        if (sample.timeSeconds < rangeStartTime ||
            sample.timeSeconds > endTime) {
            continue;
        }
        for (const PlotSeries &item : series) {
            const double value = item.value(sample);
            if (std::isfinite(value)) {
                visibleValues.append(value);
            }
        }
    }

    double yMinimum = options.robustYRange
        ? percentile(visibleValues, 0.01)
        : percentile(visibleValues, 0.0);
    double yMaximum = options.robustYRange
        ? percentile(visibleValues, 0.99)
        : percentile(visibleValues, 1.0);
    normalizeRange(yMinimum, yMaximum, options.robustYRange,
                   options.symmetricYRange);
    if (options.yMinimum.has_value()) {
        yMinimum = *options.yMinimum;
    }
    if (options.yMaximum.has_value()) {
        yMaximum = *options.yMaximum;
    }
    if (!(yMaximum > yMinimum)) {
        yMinimum = -1.0;
        yMaximum = 1.0;
    }

    constexpr int horizontalDivisions = 5;
    constexpr int verticalDivisions = 7;
    for (int index = 0; index <= horizontalDivisions; ++index) {
        const qreal ratio = static_cast<qreal>(index) / horizontalDivisions;
        const qreal y = plotArea.bottom() - ratio * plotArea.height();
        painter.setPen(QPen(QColor(230, 230, 230), 1.0));
        painter.drawLine(QPointF(plotArea.left(), y),
                         QPointF(plotArea.right(), y));
        painter.setPen(QColor(80, 80, 80));
        const double value = yMinimum + ratio * (yMaximum - yMinimum);
        painter.drawText(QRectF(bounds.left() + 8.0, y - 11.0,
                                84.0, 22.0),
                         Qt::AlignRight | Qt::AlignVCenter,
                         QString::number(value, 'f', 1));
    }
    for (int index = 0; index <= verticalDivisions; ++index) {
        const qreal ratio = static_cast<qreal>(index) / verticalDivisions;
        const qreal x = plotArea.left() + ratio * plotArea.width();
        painter.setPen(QPen(QColor(230, 230, 230), 1.0));
        painter.drawLine(QPointF(x, plotArea.top()),
                         QPointF(x, plotArea.bottom()));
        painter.setPen(QColor(80, 80, 80));
        const double time = startTime + ratio * (endTime - startTime);
        painter.drawText(QRectF(x - 42.0, plotArea.bottom() + 7.0,
                                84.0, 22.0),
                         Qt::AlignHCenter | Qt::AlignTop,
                         QString::number(time, 'f',
                             endTime - startTime <= 1.0 ? 2 : 1));
    }

    if (yMinimum < 0.0 && yMaximum > 0.0) {
        const qreal zeroY = plotArea.bottom()
            - ((0.0 - yMinimum) / (yMaximum - yMinimum)) * plotArea.height();
        painter.setPen(QPen(QColor(145, 145, 145), 1.2, Qt::DashLine));
        painter.drawLine(QPointF(plotArea.left(), zeroY),
                         QPointF(plotArea.right(), zeroY));
    }

    painter.setPen(QPen(QColor(65, 65, 65), 1.3));
    painter.drawRect(plotArea);
    painter.drawText(QRectF(plotArea.left(), bounds.bottom() - 28.0,
                            plotArea.width(), 22.0),
                     Qt::AlignCenter, translated("时间 (s)"));
    painter.save();
    painter.translate(bounds.left() + 22.0, plotArea.center().y());
    painter.rotate(-90.0);
    painter.drawText(QRectF(-plotArea.height() / 2.0, -12.0,
                            plotArea.height(), 24.0),
                     Qt::AlignCenter, verticalUnit);
    painter.restore();

    painter.setClipRect(plotArea.adjusted(1.0, 1.0, -1.0, -1.0));
    for (const PlotSeries &item : series) {
        QPainterPath path;
        bool firstPoint = true;
        for (const MotorSample &sample : samples) {
            if (sample.timeSeconds < startTime ||
                sample.timeSeconds > endTime) {
                continue;
            }
            const double value = item.value(sample);
            if (!std::isfinite(value)) {
                firstPoint = true;
                continue;
            }
            const qreal x = plotArea.left()
                + ((sample.timeSeconds - startTime) / (endTime - startTime))
                    * plotArea.width();
            const qreal y = plotArea.bottom()
                - ((value - yMinimum) / (yMaximum - yMinimum))
                    * plotArea.height();
            if (firstPoint) {
                path.moveTo(x, y);
                firstPoint = false;
            } else {
                path.lineTo(x, y);
            }
        }
        QPen seriesPen(item.color);
        seriesPen.setStyle(item.penStyle);
        seriesPen.setWidthF(item.penWidth);
        seriesPen.setCapStyle(Qt::RoundCap);
        seriesPen.setJoinStyle(Qt::RoundJoin);
        painter.setPen(seriesPen);
        painter.drawPath(path);
    }
    painter.restore();
}

void drawMetadata(QPainter &painter, MotorControlMode mode,
                  const QString &pidSummary,
                  const SpeedTestMetrics &speed,
                  const PositionTestMetrics &position)
{
    QFont heading = painter.font();
    heading.setPointSize(19);
    heading.setBold(true);
    painter.setFont(heading);
    painter.setPen(QColor(25, 25, 25));
    painter.drawText(QRectF(25.0, 10.0, 1750.0, 38.0), Qt::AlignCenter,
                     mode == MotorControlMode::Speed
                         ? translated("速度 PID 7 秒跟踪测试")
                         : translated("位置 PID 7 秒跟踪测试"));

    QFont metadata = painter.font();
    metadata.setPointSize(9);
    metadata.setBold(false);
    painter.setFont(metadata);
    painter.setPen(QColor(75, 75, 75));
    painter.drawText(QRectF(30.0, 48.0, 1740.0, 30.0),
                     Qt::AlignCenter, pidSummary);

    const QRectF metricBounds(20.0, 82.0, 1760.0, 128.0);
    painter.setPen(QPen(QColor(195, 195, 195), 1.0));
    painter.setBrush(Qt::white);
    painter.drawRoundedRect(metricBounds, 5.0, 5.0);

    QFont metricFont = painter.font();
    metricFont.setPointSize(11);
    painter.setFont(metricFont);
    painter.setPen(QColor(45, 45, 45));

    QStringList metrics;
    if (mode == MotorControlMode::Speed) {
        metrics = {
            translated("Target: ") +
                formatMetric(speed.targetRpm, 1, translated(" RPM")),
            translated("Samples: ") + QString::number(speed.sampleCount),
            translated("Sample period: ") +
                formatMetric(speed.samplePeriodMs, 3, translated(" ms")),
            translated("Rise time: ") +
                formatMetric(speed.riseTimeSeconds, 3, translated(" s")),
            translated("Overshoot: ") +
                formatMetric(speed.overshootPercent, 2, translated(" %")),
            translated("Settling time: ") +
                formatMetric(speed.settlingTimeSeconds, 3, translated(" s")),
            translated("Steady error (MAE): ") +
                formatMetric(speed.steadyStateErrorRpm, 2,
                             translated(" RPM")),
            translated("RMSE: ") +
                formatMetric(speed.rmseRpm, 2, translated(" RPM")),
            translated("Maximum error: ") +
                formatMetric(speed.maximumErrorRpm, 2, translated(" RPM"))
        };
    } else {
        metrics = {
            translated("Target: ") +
                formatMetric(position.targetPositionDeg, 2,
                             translated(" deg")),
            translated("Samples: ") + QString::number(position.sampleCount),
            translated("Sample period: ") +
                formatMetric(position.samplePeriodMs, 3, translated(" ms")),
            translated("Rise time: ") +
                formatMetric(position.riseTimeSeconds, 3, translated(" s")),
            translated("Overshoot: ") +
                formatMetric(position.overshootPercent, 2, translated(" %")),
            translated("Settling time: ") +
                formatMetric(position.settlingTimeSeconds, 3,
                             translated(" s")),
            translated("Steady error (MAE): ") +
                formatMetric(position.steadyStateErrorDeg, 3,
                             translated(" deg")),
            translated("RMSE: ") +
                formatMetric(position.rmseDeg, 3, translated(" deg")),
            translated("Maximum error: ") +
                formatMetric(position.maximumErrorDeg, 3,
                             translated(" deg"))
        };
    }

    constexpr int columns = 3;
    constexpr int rows = 3;
    const qreal cellWidth = metricBounds.width() / columns;
    const qreal cellHeight = metricBounds.height() / rows;
    for (int index = 0; index < metrics.size(); ++index) {
        const int row = index / columns;
        const int column = index % columns;
        painter.drawText(
            QRectF(metricBounds.left() + column * cellWidth + 24.0,
                   metricBounds.top() + row * cellHeight,
                   cellWidth - 40.0, cellHeight),
            Qt::AlignLeft | Qt::AlignVCenter, metrics[index]);
    }
}
}

double MotorPlotRenderer::shortestAngleErrorDeg(double targetDeg,
                                                double currentDeg)
{
    if (!std::isfinite(targetDeg) || !std::isfinite(currentDeg)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double error = std::fmod(targetDeg - currentDeg + 180.0, 360.0);
    if (error < 0.0) {
        error += 360.0;
    }
    return error - 180.0;
}

SpeedTestMetrics MotorPlotRenderer::calculateSpeedMetrics(
    const QVector<MotorSample> &samples)
{
    SpeedTestMetrics metrics;
    metrics.sampleCount = static_cast<int>(samples.size());
    metrics.samplePeriodMs = medianSamplePeriodMs(samples);
    if (samples.isEmpty()) {
        return metrics;
    }

    const qsizetype tailCount = std::max<qsizetype>(1, samples.size() / 5);
    metrics.targetRpm = median(sampleSlice(
        samples, samples.size() - tailCount, samples.size(),
        [](const MotorSample &sample) { return sample.referenceSpeedRpm; }));
    const double targetMagnitude = std::abs(metrics.targetRpm);
    if (!(targetMagnitude > kMinimumMagnitude)) {
        return metrics;
    }
    const double direction = metrics.targetRpm >= 0.0 ? 1.0 : -1.0;

    double time10 = std::numeric_limits<double>::quiet_NaN();
    double time90 = std::numeric_limits<double>::quiet_NaN();
    double directedPeak = -std::numeric_limits<double>::infinity();
    int trackingStart = -1;
    for (qsizetype index = 0; index < samples.size(); ++index) {
        const MotorSample &sample = samples[index];
        const double directedMeasured = sample.measuredSpeedRpm * direction;
        const double directedReference = sample.referenceSpeedRpm * direction;
        directedPeak = std::max(directedPeak, directedMeasured);
        if (!std::isfinite(time10) && directedMeasured >= 0.1 * targetMagnitude) {
            time10 = sample.timeSeconds;
        }
        if (std::isfinite(time10) && !std::isfinite(time90) &&
            directedMeasured >= 0.9 * targetMagnitude) {
            time90 = sample.timeSeconds;
        }
        if (trackingStart < 0 && directedReference >= 0.9 * targetMagnitude) {
            trackingStart = static_cast<int>(index);
        }
    }
    if (std::isfinite(time10) && std::isfinite(time90) && time90 >= time10) {
        metrics.riseTimeSeconds = time90 - time10;
    }
    if (std::isfinite(directedPeak)) {
        metrics.overshootPercent = std::max(
            0.0, (directedPeak - targetMagnitude) / targetMagnitude * 100.0);
    }

    const double settlingBand = std::max(0.02 * targetMagnitude, 5.0);
    int lastOutside = -1;
    for (qsizetype index = 0; index < samples.size(); ++index) {
        if (std::abs(samples[index].measuredSpeedRpm - metrics.targetRpm) >
            settlingBand) {
            lastOutside = static_cast<int>(index);
        }
    }
    if (lastOutside < samples.size() - 1) {
        const int settledIndex = std::max(0, lastOutside + 1);
        metrics.settlingTimeSeconds = samples[settledIndex].timeSeconds;
    }

    const double steadyStart = samples.constLast().timeSeconds - 1.0;
    double steadyAbsoluteError = 0.0;
    int steadyCount = 0;
    for (const MotorSample &sample : samples) {
        if (sample.timeSeconds >= steadyStart) {
            steadyAbsoluteError +=
                std::abs(sample.referenceSpeedRpm - sample.measuredSpeedRpm);
            ++steadyCount;
        }
    }
    if (steadyCount > 0) {
        metrics.steadyStateErrorRpm = steadyAbsoluteError / steadyCount;
    }

    // RMSE 与最大误差从参考速度首次达到最终目标的 90% 后开始计算，
    // 避免把命令斜坡本身当作控制器的稳态跟踪误差。
    if (trackingStart >= 0) {
        double squaredError = 0.0;
        double maximumError = 0.0;
        int count = 0;
        for (qsizetype index = trackingStart; index < samples.size(); ++index) {
            const double error = samples[index].referenceSpeedRpm -
                                 samples[index].measuredSpeedRpm;
            squaredError += error * error;
            maximumError = std::max(maximumError, std::abs(error));
            ++count;
        }
        if (count > 0) {
            metrics.rmseRpm = std::sqrt(squaredError / count);
            metrics.maximumErrorRpm = maximumError;
        }
    }
    return metrics;
}

PositionTestMetrics MotorPlotRenderer::calculatePositionMetrics(
    const QVector<MotorSample> &samples)
{
    PositionTestMetrics metrics;
    metrics.sampleCount = static_cast<int>(samples.size());
    metrics.samplePeriodMs = medianSamplePeriodMs(samples);
    if (samples.isEmpty()) {
        return metrics;
    }

    // 初始角度只取最前 10 个样本（约 20 ms），避免把已经发生的阶跃运动
    // 混入初始位置估计。
    const qsizetype headCount = std::min<qsizetype>(10, samples.size());
    const qsizetype tailCount = std::max<qsizetype>(1, samples.size() / 5);
    metrics.initialPositionDeg = circularMeanDeg(sampleSlice(
        samples, 0, headCount,
        [](const MotorSample &sample) { return sample.currentPositionDeg; }));
    metrics.targetPositionDeg = circularMeanDeg(sampleSlice(
        samples, samples.size() - tailCount, samples.size(),
        [](const MotorSample &sample) { return sample.targetPositionDeg; }));

    const double signedStep = shortestAngleErrorDeg(
        metrics.targetPositionDeg, metrics.initialPositionDeg);
    const double stepMagnitude = std::abs(signedStep);
    if (!(stepMagnitude > kMinimumMagnitude)) {
        return metrics;
    }
    const double direction = signedStep >= 0.0 ? 1.0 : -1.0;
    double time10 = std::numeric_limits<double>::quiet_NaN();
    double time90 = std::numeric_limits<double>::quiet_NaN();
    double directedPeak = -std::numeric_limits<double>::infinity();
    int trackingStart = -1;
    for (qsizetype index = 0; index < samples.size(); ++index) {
        const MotorSample &sample = samples[index];
        const double progress = shortestAngleErrorDeg(
            sample.currentPositionDeg, metrics.initialPositionDeg) * direction;
        const double referenceProgress = shortestAngleErrorDeg(
            sample.targetPositionDeg, metrics.initialPositionDeg) * direction;
        directedPeak = std::max(directedPeak, progress);
        if (!std::isfinite(time10) && progress >= 0.1 * stepMagnitude) {
            time10 = sample.timeSeconds;
        }
        if (std::isfinite(time10) && !std::isfinite(time90) &&
            progress >= 0.9 * stepMagnitude) {
            time90 = sample.timeSeconds;
        }
        if (trackingStart < 0 && referenceProgress >= 0.9 * stepMagnitude) {
            trackingStart = static_cast<int>(index);
        }
    }
    if (std::isfinite(time10) && std::isfinite(time90) && time90 >= time10) {
        metrics.riseTimeSeconds = time90 - time10;
    }
    if (std::isfinite(directedPeak)) {
        metrics.overshootPercent = std::max(
            0.0, (directedPeak - stepMagnitude) / stepMagnitude * 100.0);
    }

    const double settlingBand = std::max(0.02 * stepMagnitude, 0.5);
    int lastOutside = -1;
    for (qsizetype index = 0; index < samples.size(); ++index) {
        const double error = shortestAngleErrorDeg(
            metrics.targetPositionDeg, samples[index].currentPositionDeg);
        if (std::abs(error) > settlingBand) {
            lastOutside = static_cast<int>(index);
        }
    }
    if (lastOutside < samples.size() - 1) {
        const int settledIndex = std::max(0, lastOutside + 1);
        metrics.settlingTimeSeconds = samples[settledIndex].timeSeconds;
    }

    const double steadyStart = samples.constLast().timeSeconds - 1.0;
    double steadyAbsoluteError = 0.0;
    int steadyCount = 0;
    for (const MotorSample &sample : samples) {
        if (sample.timeSeconds >= steadyStart) {
            steadyAbsoluteError += std::abs(shortestAngleErrorDeg(
                sample.targetPositionDeg, sample.currentPositionDeg));
            ++steadyCount;
        }
    }
    if (steadyCount > 0) {
        metrics.steadyStateErrorDeg = steadyAbsoluteError / steadyCount;
    }

    if (trackingStart >= 0) {
        double squaredError = 0.0;
        double maximumError = 0.0;
        int count = 0;
        for (qsizetype index = trackingStart; index < samples.size(); ++index) {
            const double error = shortestAngleErrorDeg(
                samples[index].targetPositionDeg,
                samples[index].currentPositionDeg);
            squaredError += error * error;
            maximumError = std::max(maximumError, std::abs(error));
            ++count;
        }
        if (count > 0) {
            metrics.rmseDeg = std::sqrt(squaredError / count);
            metrics.maximumErrorDeg = maximumError;
        }
    }
    return metrics;
}

bool MotorPlotRenderer::savePng(const QVector<MotorSample> &samples,
                                MotorControlMode mode,
                                const QString &pidSummary,
                                const QString &filePath,
                                QString *errorMessage)
{
    if (samples.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = translated("没有可保存的测试采样数据");
        }
        return false;
    }

    const SpeedTestMetrics speed = calculateSpeedMetrics(samples);
    const PositionTestMetrics position = calculatePositionMetrics(samples);
    QImage image(kReportWidth, kReportHeight,
                 QImage::Format_ARGB32_Premultiplied);
    image.fill(QColor(248, 249, 251));
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);
    drawMetadata(painter, mode, pidSummary, speed, position);

    const double actualEnd = std::max(0.001, samples.constLast().timeSeconds);
    const double settlingTime = mode == MotorControlMode::Speed
        ? speed.settlingTimeSeconds : position.settlingTimeSeconds;
    const double transientEnd = std::min(
        actualEnd,
        std::isfinite(settlingTime) && settlingTime <= 0.45 ? 0.5 : 1.0);
    const QVector<PlotSeries> iqSeries = {
        memberSeries(translated("Iq 目标"), QColor(255, 143, 0),
                     &MotorSample::iqReferenceMa, Qt::DashLine, 1.5),
        memberSeries(translated("Iq 实测"), QColor(211, 47, 47),
                     &MotorSample::iqMa, Qt::SolidLine, 2.5)
    };

    constexpr qreal panelLeft = 20.0;
    constexpr qreal panelWidth = 1760.0;
    constexpr qreal panelHeight = 420.0;
    constexpr qreal firstPanelTop = 225.0;
    constexpr qreal panelStep = 435.0;
    const PlotOptions overallOptions{0.0, actualEnd};
    const PlotOptions transientOptions{0.0, transientEnd};
    PlotOptions errorOptions{0.0, actualEnd};
    // 启动瞬态已经在第二面板单独放大。误差面板仍绘制全部原始点，
    // 但用瞬态窗口之后的数据确定对称 Y 轴，启动大误差允许被 clip，
    // 从而让几 RPM/小角度的稳态误差与周期波动保持可见。
    errorOptions.yRangeStartTimeSeconds = transientEnd;
    errorOptions.robustYRange = true;
    errorOptions.symmetricYRange = true;
    PlotOptions currentOptions{0.0, actualEnd};
    currentOptions.robustYRange = true;

    if (mode == MotorControlMode::Speed) {
        const QVector<PlotSeries> speedSeries = {
            memberSeries(translated("参考速度"), QColor(244, 81, 30),
                         &MotorSample::referenceSpeedRpm,
                         Qt::DashLine, 1.5),
            memberSeries(translated("实测速度"), QColor(0, 137, 123),
                         &MotorSample::measuredSpeedRpm,
                         Qt::SolidLine, 2.5)
        };
        const QVector<PlotSeries> speedErrorSeries = {
            {translated("参考 - 实测"), QColor(94, 53, 177),
             [](const MotorSample &sample) {
                 return sample.referenceSpeedRpm - sample.measuredSpeedRpm;
             }, Qt::SolidLine, 2.2}
        };
        drawPanel(painter,
                  QRectF(panelLeft, firstPanelTop,
                         panelWidth, panelHeight),
                  samples, translated("速度总体跟踪"),
                  translated("速度 (RPM)"), speedSeries, overallOptions);
        drawPanel(painter,
                  QRectF(panelLeft, firstPanelTop + panelStep,
                         panelWidth, panelHeight),
                  samples, translated("启动瞬态"),
                  translated("速度 (RPM)"), speedSeries, transientOptions);
        drawPanel(painter,
                  QRectF(panelLeft, firstPanelTop + panelStep * 2.0,
                         panelWidth, panelHeight),
                  samples, translated("速度误差"), translated("误差 (RPM)"),
                  speedErrorSeries, errorOptions);
    } else {
        const QVector<PlotSeries> positionSeries = {
            memberSeries(translated("目标位置"), QColor(198, 40, 40),
                         &MotorSample::targetPositionDeg,
                         Qt::DashLine, 1.5),
            memberSeries(translated("实测位置"), QColor(21, 101, 192),
                         &MotorSample::currentPositionDeg,
                         Qt::SolidLine, 2.5)
        };
        const QVector<PlotSeries> positionErrorSeries = {
            {translated("最短角度误差"), QColor(94, 53, 177),
             [](const MotorSample &sample) {
                 return MotorPlotRenderer::shortestAngleErrorDeg(
                     sample.targetPositionDeg, sample.currentPositionDeg);
             }, Qt::SolidLine, 2.2}
        };
        drawPanel(painter,
                  QRectF(panelLeft, firstPanelTop,
                         panelWidth, panelHeight),
                  samples, translated("位置总体跟踪"),
                  translated("位置 (deg)"), positionSeries, overallOptions);
        drawPanel(painter,
                  QRectF(panelLeft, firstPanelTop + panelStep,
                         panelWidth, panelHeight),
                  samples, translated("启动 / 阶跃瞬态"),
                  translated("位置 (deg)"), positionSeries, transientOptions);
        drawPanel(painter,
                  QRectF(panelLeft, firstPanelTop + panelStep * 2.0,
                         panelWidth, panelHeight),
                  samples, translated("位置误差（最短角度）"),
                  translated("误差 (deg)"),
                  positionErrorSeries, errorOptions);
    }

    drawPanel(painter,
              QRectF(panelLeft, firstPanelTop + panelStep * 3.0,
                     panelWidth, panelHeight),
              samples, translated("q 轴电流跟踪"),
              translated("电流 (mA)"), iqSeries, currentOptions);
    painter.end();

    if (!image.save(filePath, "PNG")) {
        if (errorMessage != nullptr) {
            *errorMessage = translated("PNG 文件写入失败");
        }
        return false;
    }
    return true;
}

bool MotorPlotRenderer::saveCsv(const QVector<MotorSample> &samples,
                                const QString &filePath,
                                QString *errorMessage)
{
    if (samples.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = translated("没有可保存的测试采样数据");
        }
        return false;
    }

    QSaveFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = translated("CSV 文件打开失败：") +
                            file.errorString();
        }
        return false;
    }
    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(6);
    stream << QStringLiteral(
        "time_s,speed_rpm,speed_ref_rpm,speed_error_rpm,"
        "position_deg,position_ref_deg,position_error_deg,"
        "iq_ma,iq_ref_ma,id_ma,id_ref_ma\n");
    for (const MotorSample &sample : samples) {
        stream << sample.timeSeconds << ','
               << sample.measuredSpeedRpm << ','
               << sample.referenceSpeedRpm << ','
               << sample.referenceSpeedRpm - sample.measuredSpeedRpm << ','
               << sample.currentPositionDeg << ','
               << sample.targetPositionDeg << ','
               << shortestAngleErrorDeg(sample.targetPositionDeg,
                                        sample.currentPositionDeg) << ','
               << sample.iqMa << ','
               << sample.iqReferenceMa << ','
               << sample.idMa << ','
               << sample.idReferenceMa << '\n';
    }
    stream.flush();
    if (stream.status() != QTextStream::Ok) {
        file.cancelWriting();
        if (errorMessage != nullptr) {
            *errorMessage = translated("CSV 数据写入失败");
        }
        return false;
    }
    if (!file.commit()) {
        if (errorMessage != nullptr) {
            *errorMessage = translated("CSV 文件提交失败：") +
                            file.errorString();
        }
        return false;
    }
    return true;
}
