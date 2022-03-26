#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QPainter>

#include "covsearch/types.hpp"

#pragma once

namespace vis {

class RobotItem : public QGraphicsItem {
    friend class Dialog;

public:
    RobotItem();
    QRectF boundingRect() const;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
        QWidget* widget);

    void SetCurrentPlan(cs::Plan& plan);

    // state update callback
    void ConnectStateUpdateCB(cs::StateUpdateCB cb);

protected:
    void advance(int phase);

private:
    cs::Plan currentPlan_; // not used anymore
    int planIdx_;
    qreal angle_;
    qreal speed_;
    cs::RobotState currentPos_;

    cs::StateUpdateCB cbUpdateRobotPosition_;
};

} // namespace vis
