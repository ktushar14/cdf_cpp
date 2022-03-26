#include "qt/robot_item.hpp"
#include "covsearch/constants.hpp"
#include "covsearch/types.hpp"

#include <QDebug>
#include <QtMath>

namespace vis {

RobotItem::RobotItem()
{
    setTransformOriginPoint(QPointF(0.5 * kOneCellPx, 0.5 * kOneCellPx));

    setRotation(0);

    // set speed
    speed_ = 5;

    // start location
    int startX = 4 * kOneCellPx;
    int startY = 4 * kOneCellPx;
    setPos(mapToParent(startX, startY));
}

QRectF RobotItem::boundingRect() const
{
    return QRectF(0, 0, kOneCellPx, kOneCellPx);
}

void RobotItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
    QWidget* widget)
{
    // // Draw bounding rect
    // QRectF rect = boundingRect();
    // painter->drawRect(rect);

    QColor robotColor = QColor(195, 245, 92, 200);
    painter->setBrush(QBrush(robotColor));
    painter->drawEllipse(0, 0, kOneCellPx, kOneCellPx);

    QLineF headingDirection(QPointF(0.5 * kOneCellPx, 0.5 * kOneCellPx),
        QPointF(kOneCellPx, 0.5 * kOneCellPx));
    painter->setPen(QPen(Qt::black));
    painter->drawLine(headingDirection);
}

void RobotItem::SetCurrentPlan(cs::Plan& plan)
{
    printf("RobotItem::SetCurrentPlan\n");
    printf("plan size: %d\n", (int)plan.size());
    currentPlan_ = plan;
    // for (auto s : currentPlan_) {
    //   printf("  %f, %f, %f \n", s[0], s[1], s[2]);
    // }
    planIdx_ = 0;
}

void RobotItem::ConnectStateUpdateCB(cs::StateUpdateCB cb)
{
    cbUpdateRobotPosition_ = cb;
}

void RobotItem::advance(int phase)
{
    if (!phase)
        return;

    // update robot position
    cs::RobotState updatedState;
    cbUpdateRobotPosition_(updatedState);

    if (updatedState.empty()) {
        printf("RoboItem::advance: Robot state not available\n");
        return;
    }

    setRotation(qRadiansToDegrees(updatedState[2]));
    setPos(updatedState[0], updatedState[1]);
    // printf("  position: (%f, %f)\n", (pos().x() / kOneCellPx), (pos().y() / kOneCellPx));
}

} // namespace vis
