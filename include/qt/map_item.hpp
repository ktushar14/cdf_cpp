#include "covsearch/types.hpp"

#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QPainter>

#include <memory>

#pragma once

namespace vis {

class MapItem : public QGraphicsItem {
    friend class Dialog;

public:
    MapItem(int rows, int cols);
    QRectF boundingRect() const;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
        QWidget* widget);

    void SetMap(cs::MAP_t mapPtr);
    void SetCurrentPlan(cs::Plan& plan);

    void ConnectMapUpdateCB(cs::MapUpdateCB cb);
    void ConnectGoalUpdateCB(cs::StateUpdateCB cb);

    void drawCoverageMap(QPainter* p);
    void drawCurrentRobotPlan(QPainter* p);
    void drawCurrentGoal(QPainter* p);

    void drawDiscRect(QPainter* p, const int& x, const int& y, const int& w, const int& h);
    void drawDiscCell(QPainter* p, const int& x, const int& y);
    void drawContPoint(QPainter* p, const double& x, const double& y);

    // cs::BooleanMap coverageMap_;
    cs::MAP_t map_;
    std::unique_ptr<QImage> m_map_img;
    std::unique_ptr<QPixmap> m_map_pix;
    int rows_;
    int cols_;

    cs::Plan currentPlan_;
    std::vector<QPointF> currentPlanPolyline_;

    cs::RobotState currentGoal_;

    // Callbacks
    cs::MapUpdateCB cbUpdateMap_;
    cs::StateUpdateCB cbUpdateGoal_;

protected:
    void advance(int phase);
};

} // namespace vis
