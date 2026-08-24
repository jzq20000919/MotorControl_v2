#include "plotwidget.h"

#include <QColor>
#include <QFontMetrics>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>

#include <algorithm>
#include <cmath>
#include <limits>

MotorPlotWidget::MotorPlotWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(760, 520);
    setAutoFillBackground(true);
    QPalette plotPalette = palette();
    plotPalette.setColor(QPalette::Window, QColor(250, 250, 250));
    setPalette(plotPalette);
}

void MotorPlotWidget::clearData()
{
    samples_.clear();
    update();
}

void MotorPlotWidget::setData(const QVector<MotorSample> &samples,
                              MotorControlMode mode)
{
    samples_ = samples;
    mode_ = mode;
    update();
}

void MotorPlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(250, 250, 250));

    if (samples_.isEmpty()) {
        painter.setPen(QColor(90, 90, 90));
        painter.drawText(rect(), Qt::AlignCenter,
                         tr("电机停止后将在这里显示本次运行曲线"));
        return;
    }

    const qreal gap = 14.0;
    const qreal margin = 8.0;
    const qreal panelHeight = (height() - margin * 2.0 - gap) / 2.0;
    const QRectF currentBounds(margin, margin,
                               width() - margin * 2.0, panelHeight);
    const QRectF modeBounds(margin, margin + panelHeight + gap,
                            width() - margin * 2.0, panelHeight);

    const QVector<PlotSeries> currentSeries = {
        {tr("Iq 实测"), QColor(211, 47, 47), &MotorSample::iqMa},
        {tr("Iq 目标"), QColor(255, 143, 0), &MotorSample::iqReferenceMa},
        {tr("Id 实测"), QColor(25, 118, 210), &MotorSample::idMa},
        {tr("Id 目标"), QColor(123, 31, 162), &MotorSample::idReferenceMa}
    };
    drawPanel(painter, currentBounds, tr("dq 轴电流"), tr("电流 (mA)"),
              currentSeries);

    if (mode_ == MotorControlMode::Speed) {
        const QVector<PlotSeries> speedSeries = {
            {tr("实测速度"), QColor(0, 137, 123),
             &MotorSample::measuredSpeedRpm},
            {tr("参考速度"), QColor(244, 81, 30),
             &MotorSample::referenceSpeedRpm}
        };
        drawPanel(painter, modeBounds, tr("速度控制"), tr("速度 (RPM)"),
                  speedSeries);
    } else {
        const QVector<PlotSeries> positionSeries = {
            {tr("实测位置"), QColor(21, 101, 192),
             &MotorSample::currentPositionDeg},
            {tr("目标位置"), QColor(198, 40, 40),
             &MotorSample::targetPositionDeg}
        };
        drawPanel(painter, modeBounds, tr("位置控制"), tr("位置 (°)"),
                  positionSeries);
    }
}

void MotorPlotWidget::drawPanel(
    QPainter &painter,
    const QRectF &bounds,
    const QString &title,
    const QString &verticalUnit,
    const QVector<PlotSeries> &series) const
{
    painter.save();
    painter.setPen(QPen(QColor(205, 205, 205), 1.0));
    painter.setBrush(Qt::white);
    painter.drawRoundedRect(bounds, 4.0, 4.0);

    const qreal left = bounds.left() + 78.0;
    const qreal right = bounds.right() - 18.0;
    const qreal top = bounds.top() + 48.0;
    const qreal bottom = bounds.bottom() - 43.0;
    const QRectF plotArea(left, top, std::max<qreal>(1.0, right - left),
                          std::max<qreal>(1.0, bottom - top));

    painter.setPen(QColor(35, 35, 35));
    QFont titleFont = painter.font();
    titleFont.setBold(true);
    painter.setFont(titleFont);
    painter.drawText(QRectF(bounds.left() + 12.0, bounds.top() + 8.0,
                            145.0, 24.0),
                     Qt::AlignLeft | Qt::AlignVCenter, title);

    QFont legendFont = painter.font();
    legendFont.setBold(false);
    legendFont.setPointSize(std::max(8, legendFont.pointSize() - 1));
    painter.setFont(legendFont);
    qreal legendX = bounds.left() + 160.0;
    const qreal legendY = bounds.top() + 20.0;
    for (const PlotSeries &item : series) {
        painter.setPen(QPen(item.color, 3.0));
        painter.drawLine(QPointF(legendX, legendY),
                         QPointF(legendX + 20.0, legendY));
        painter.setPen(QColor(55, 55, 55));
        painter.drawText(QPointF(legendX + 25.0, legendY + 4.0), item.name);
        legendX += 33.0 + painter.fontMetrics().horizontalAdvance(item.name);
    }

    double yMinimum = std::numeric_limits<double>::max();
    double yMaximum = std::numeric_limits<double>::lowest();
    for (const MotorSample &sample : samples_) {
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

    const double timeMaximum = std::max(1.0, samples_.constLast().timeSeconds);
    constexpr int horizontalDivisions = 5;
    constexpr int verticalDivisions = 6;
    painter.setFont(legendFont);
    for (int index = 0; index <= horizontalDivisions; ++index) {
        const qreal ratio = static_cast<qreal>(index) / horizontalDivisions;
        const qreal y = plotArea.bottom() - ratio * plotArea.height();
        painter.setPen(QPen(QColor(230, 230, 230), 1.0));
        painter.drawLine(QPointF(plotArea.left(), y),
                         QPointF(plotArea.right(), y));
        painter.setPen(QColor(80, 80, 80));
        const double value = yMinimum + ratio * (yMaximum - yMinimum);
        painter.drawText(QRectF(bounds.left() + 5.0, y - 10.0,
                                plotArea.left() - bounds.left() - 12.0, 20.0),
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
        painter.drawText(QRectF(x - 35.0, plotArea.bottom() + 5.0,
                                70.0, 18.0),
                         Qt::AlignHCenter | Qt::AlignTop,
                         QString::number(ratio * timeMaximum, 'f', 1));
    }

    painter.setPen(QPen(QColor(75, 75, 75), 1.2));
    painter.drawRect(plotArea);
    painter.drawText(QRectF(plotArea.left(), bounds.bottom() - 21.0,
                            plotArea.width(), 18.0),
                     Qt::AlignCenter, tr("时间 (s)"));

    painter.save();
    painter.translate(bounds.left() + 16.0, plotArea.center().y());
    painter.rotate(-90.0);
    painter.drawText(QRectF(-plotArea.height() / 2.0, -10.0,
                            plotArea.height(), 20.0),
                     Qt::AlignCenter, verticalUnit);
    painter.restore();

    painter.setClipRect(plotArea.adjusted(1.0, 1.0, -1.0, -1.0));
    for (const PlotSeries &item : series) {
        QPainterPath path;
        bool firstPoint = true;
        for (const MotorSample &sample : samples_) {
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
        painter.setPen(QPen(item.color, 1.8));
        painter.drawPath(path);
    }
    painter.restore();
}
