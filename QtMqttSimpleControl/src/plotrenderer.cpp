#include "plotrenderer.h"

#include <QColor>
#include <QCoreApplication>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QRectF>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
struct PlotSeries
{
    QString name;
    QColor color;
    double MotorSample::*value = nullptr;
};

QString translated(const char *text)
{
    return QCoreApplication::translate("MotorPlotRenderer", text);
}

void drawPanel(QPainter &painter, const QRectF &bounds,
               const QVector<MotorSample> &samples,
               const QString &title, const QString &verticalUnit,
               const QVector<PlotSeries> &series)
{
    painter.save();
    painter.setPen(QPen(QColor(195, 195, 195), 1.0));
    painter.setBrush(Qt::white);
    painter.drawRoundedRect(bounds, 5.0, 5.0);

    const QRectF plotArea(bounds.left() + 100.0, bounds.top() + 64.0,
                          bounds.width() - 130.0, bounds.height() - 125.0);

    QFont titleFont = painter.font();
    titleFont.setPointSize(14);
    titleFont.setBold(true);
    painter.setFont(titleFont);
    painter.setPen(QColor(35, 35, 35));
    painter.drawText(QRectF(bounds.left() + 18.0, bounds.top() + 10.0,
                            210.0, 30.0),
                     Qt::AlignLeft | Qt::AlignVCenter, title);

    QFont axisFont = painter.font();
    axisFont.setPointSize(9);
    axisFont.setBold(false);
    painter.setFont(axisFont);
    qreal legendX = bounds.left() + 240.0;
    const qreal legendY = bounds.top() + 28.0;
    for (const PlotSeries &item : series) {
        painter.setPen(QPen(item.color, 4.0));
        painter.drawLine(QPointF(legendX, legendY),
                         QPointF(legendX + 26.0, legendY));
        painter.setPen(QColor(55, 55, 55));
        painter.drawText(QPointF(legendX + 34.0, legendY + 5.0), item.name);
        legendX += 48.0 + painter.fontMetrics().horizontalAdvance(item.name);
    }

    double yMinimum = std::numeric_limits<double>::max();
    double yMaximum = std::numeric_limits<double>::lowest();
    for (const MotorSample &sample : samples) {
        for (const PlotSeries &item : series) {
            const double value = sample.*(item.value);
            yMinimum = std::min(yMinimum, value);
            yMaximum = std::max(yMaximum, value);
        }
    }
    if (!std::isfinite(yMinimum) || !std::isfinite(yMaximum)) {
        yMinimum = -1.0;
        yMaximum = 1.0;
    } else if (std::abs(yMaximum - yMinimum) < 1.0e-9) {
        const double padding = std::max(1.0, std::abs(yMaximum) * 0.1);
        yMinimum -= padding;
        yMaximum += padding;
    } else {
        const double padding = (yMaximum - yMinimum) * 0.08;
        yMinimum -= padding;
        yMaximum += padding;
    }
    const double timeMaximum = std::max(7.0, samples.constLast().timeSeconds);

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
        painter.drawText(QRectF(x - 38.0, plotArea.bottom() + 7.0,
                                76.0, 22.0),
                         Qt::AlignHCenter | Qt::AlignTop,
                         QString::number(ratio * timeMaximum, 'f', 1));
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
            const double value = sample.*(item.value);
            const qreal x = plotArea.left()
                + (sample.timeSeconds / timeMaximum) * plotArea.width();
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
        painter.setPen(QPen(item.color, 2.4));
        painter.drawPath(path);
    }
    painter.restore();
}
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

    QImage image(1800, 1120, QImage::Format_ARGB32_Premultiplied);
    image.fill(QColor(248, 249, 251));
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QFont heading = painter.font();
    heading.setPointSize(19);
    heading.setBold(true);
    painter.setFont(heading);
    painter.setPen(QColor(25, 25, 25));
    painter.drawText(QRectF(25.0, 12.0, 1750.0, 34.0),
                     Qt::AlignCenter,
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

    const QVector<PlotSeries> currentSeries = {
        {translated("Iq 实测"), QColor(211, 47, 47), &MotorSample::iqMa},
        {translated("Iq 目标"), QColor(255, 143, 0),
         &MotorSample::iqReferenceMa},
        {translated("Id 实测"), QColor(25, 118, 210), &MotorSample::idMa},
        {translated("Id 目标"), QColor(123, 31, 162),
         &MotorSample::idReferenceMa}
    };
    drawPanel(painter, QRectF(20.0, 84.0, 1760.0, 500.0), samples,
              translated("dq 轴电流"), translated("电流 (mA)"),
              currentSeries);

    if (mode == MotorControlMode::Speed) {
        const QVector<PlotSeries> speedSeries = {
            {translated("实测速度"), QColor(0, 137, 123),
             &MotorSample::measuredSpeedRpm},
            {translated("参考速度"), QColor(244, 81, 30),
             &MotorSample::referenceSpeedRpm}
        };
        drawPanel(painter, QRectF(20.0, 600.0, 1760.0, 500.0), samples,
                  translated("速度跟踪"), translated("速度 (RPM)"),
                  speedSeries);
    } else {
        const QVector<PlotSeries> positionSeries = {
            {translated("实测位置"), QColor(21, 101, 192),
             &MotorSample::currentPositionDeg},
            {translated("目标位置"), QColor(198, 40, 40),
             &MotorSample::targetPositionDeg}
        };
        drawPanel(painter, QRectF(20.0, 600.0, 1760.0, 500.0), samples,
                  translated("位置跟踪"), translated("位置 (°)"),
                  positionSeries);
    }
    painter.end();

    if (!image.save(filePath, "PNG")) {
        if (errorMessage != nullptr) {
            *errorMessage = translated("PNG 文件写入失败");
        }
        return false;
    }
    return true;
}
