#include "qt/map_item.hpp"
#include "covsearch/constants.hpp"
#include "covsearch/types.hpp"
#include "covsearch/helpers.hpp"

#include <fstream>

namespace vis {

MapItem::MapItem(int rows, int cols)
:
rows_(rows),
cols_(cols),
currentGoal_()
{
    m_map_img = std::make_unique<QImage>(cols_, rows_, QImage::Format_RGB32);
    m_map_pix = std::make_unique<QPixmap>(cols_, rows_);
    map_ = (cs::MAP_t)calloc(rows_ * cols_, sizeof(decltype(*map_)));
}

QRectF MapItem::boundingRect() const
{
    return QRect(0, 0, rows_ * kOneCellPx, cols_ * kOneCellPx);
}

void MapItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
    QWidget* widget)
{

    // draw MapItem bounding rect
    // QRectF rect = boundingRect();
    // QColor mapRectColor;
    // mapRectColor.setRgb(106, 137, 200);
    // painter->setPen(mapRectColor);
    // painter->drawRect(rect);

    drawCoverageMap(painter);
    drawCurrentRobotPlan(painter);
    drawCurrentGoal(painter);
}

void MapItem::SetMap(cs::MAP_t mapPtr)
{
    map_ = mapPtr;
}

void MapItem::SetCurrentPlan(cs::Plan& plan)
{
    currentPlan_.clear();
    currentPlanPolyline_.clear();
    currentPlan_ = plan;

    // TODO: make functions for
    // (1) converting cs::Plan to std::vector<QPointF>
    // (2) converting continuous point to proper resolution for visualization,
    // i.e., this: auto x = s[0] * kOneCellPx + 0.5 * kOneCellPx;
    for (auto s : currentPlan_) {
        auto x = s[0] * kOneCellPx + 0.5 * kOneCellPx;
        auto y = s[1] * kOneCellPx + 0.5 * kOneCellPx;
        QPointF point(x, y);
        currentPlanPolyline_.push_back(point);
    }
}

void MapItem::ConnectMapUpdateCB(cs::MapUpdateCB cb)
{
    cbUpdateMap_ = cb;
}

void MapItem::ConnectGoalUpdateCB(cs::StateUpdateCB cb)
{
    cbUpdateGoal_ = cb;
}

void MapItem::advance(int phase)
{
    if (!phase) {
        return;
    }
    cbUpdateMap_(map_);
    cbUpdateGoal_(currentGoal_);
}

void MapItem::drawCoverageMap(QPainter* p)
{
    QColor color;
    for (auto r = 0; r < rows_; ++r) {
    for (auto c = 0; c < cols_; ++c) {
        auto idx = GETMAPINDEX(r, c, rows_, cols_);

        if (cs::isFrontierCell(r, c, map_, rows_, cols_))
        {
            /// frontier cell
            QColor cell_color = QColor(255, 0, 0, 100);
            p->setBrush(cell_color);
            p->setPen(QColor(255, 0, 0, 125));

            auto& x = c;
            auto& y = r;
            m_map_img->setPixel(x, y, cell_color.rgb());
            // drawDiscCell(p, x, y);
        }
        else if (map_[idx] == 1)
        {
            /// traversable, covered
            QColor cell_color = QColor(0, 255, 0, 125);
            p->setBrush(cell_color); /// GREEN
            p->setPen(QColor(0, 255, 0, 125));   /// GREEN

            auto& x = c;
            auto& y = r;
            m_map_img->setPixel(x, y, cell_color.rgb());
            // drawDiscCell(p, x, y);
        }
        else if (map_[idx] == 2)
        {
            /// traversable, not covered
            QColor cell_color = QColor(255, 255, 255, 255);
            p->setBrush(cell_color); /// WHITE
            p->setPen(QColor(0, 0, 0, 10));          /// GREY

            auto& x = c;
            auto& y = r;
            m_map_img->setPixel(x, y, cell_color.rgb());
            // drawDiscCell(p, x, y);
        }
        else if (map_[idx] == 0)
        {
            /// obstacle
            QColor cell_color = QColor(0, 0, 0, 200);
            p->setBrush(cell_color); /// BLACK
            p->setPen(QColor(0, 0, 0, 200));   /// BLACK

            auto& x = c;
            auto& y = r;
            m_map_img->setPixel(x, y, cell_color.rgb());
            // drawDiscCell(p, x, y);
        }
        else
        {
            printf("    INVALID MAP CELL VALUE\n");
        }
    }
    }

    m_map_pix->convertFromImage(*m_map_img.get());
    p->drawPixmap(QRectF(kXOrigin, kYOrigin, cols_ * kOneCellPx, rows_ * kOneCellPx), *m_map_pix.get(), m_map_pix->rect());
}

void MapItem::drawCurrentRobotPlan(QPainter* p)
{
    if (currentPlanPolyline_.empty()) return;

    // draw plan
    QPen planPen(QColor(Qt::red), /* width = */ 1.0 );
    QBrush planBrush(QColor(Qt::red), Qt::SolidPattern);
    p->setPen(planPen);
    p->setBrush(planBrush);
    p->drawPolyline(currentPlanPolyline_.data(), (int)currentPlanPolyline_.size());

    // draw waypoints along plan
    QPen waypointPen(QColor(Qt::blue), /* width = */ 1.5 );
    QBrush waypointBrush(QColor(Qt::blue), Qt::SolidPattern);
    p->setPen(waypointPen);
    p->setBrush(waypointBrush);
    for (auto s : currentPlan_) {
      auto x = s[0] * kOneCellPx;
      auto y = s[1] * kOneCellPx;
      drawContPoint(p, x, y);
    }
}

void MapItem::drawCurrentGoal(QPainter* p)
{
    if (currentGoal_.empty()) return;

    QPen goalPen(QColor(0, 0, 255, 100));
    QBrush goalBrush(QColor(Qt::blue), Qt::SolidPattern);
    p->setPen(goalPen);
    p->setBrush(goalBrush);
    drawDiscCell(p, currentGoal_[0], currentGoal_[1]);
}

void MapItem::drawDiscRect(
    QPainter* p, const int& x, const int& y, const int& w, const int& h)
{
    p->drawRect(x * kOneCellPx, y * kOneCellPx, w * kOneCellPx, h * kOneCellPx);
}

void MapItem::drawDiscCell(QPainter* p, const int& x, const int& y)
{
    p->drawRect(x * kOneCellPx, y * kOneCellPx, kOneCellPx, kOneCellPx);
}

void MapItem::drawContPoint(QPainter* p, const double& x, const double& y)
{
    QPointF point(x + 0.5 * kOneCellPx, y + 0.5 * kOneCellPx);
    p->drawPoint(point);
}

} // namespace vis
