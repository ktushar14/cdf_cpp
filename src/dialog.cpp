#include "qt/dialog.h"
#include "covsearch/constants.hpp"
// #include "ui_dialog.h"

#define DRAW_GRID true

namespace vis {

Dialog::Dialog(QWidget* parent)
    // : QDialog(parent)
    // , ui_(new Ui::Dialog)
{
    // ui_->setupUi(this);
}

Dialog::~Dialog()
{
    // delete ui_;
    delete mapItem_;
    delete robotItem_;
    delete originCell_;
    delete timer_;
    delete scene_;
    // TODO: delete scene_; not required? Does Qt delete it?
}

void Dialog::SetupGraphicsScene(int& rows, int& cols)
{
    scene_ = new QGraphicsScene(this);
    scene_->setItemIndexMethod(QGraphicsScene::BspTreeIndex);

    // ui_->graphicsView->setScene(scene_);
    // ui_->graphicsView->setRenderHint(QPainter::Antialiasing);

    view_ = std::unique_ptr<QGraphicsView>(new QGraphicsView());
    view_->setScene(scene_);
    view_->setRenderHint(QPainter::Antialiasing);

    // scene_->setSceneRect(kXOrigin, kYOrigin, kSceneXLength, kSceneYLength);
    scene_->setSceneRect(kXOrigin, kYOrigin, cols * kOneCellPx, rows * kOneCellPx);
    scene_->addRect(scene_->sceneRect(), QPen(Qt::red));
    qDebug() << "scene rect:" << scene_->sceneRect();

    // outline origin cell
    originCell_ = new QGraphicsRectItem(0, 0, vis::kOneCellPx, vis::kOneCellPx);
    originCell_->setPen(QColor(Qt::red));
    scene_->addItem(originCell_);

    // ui_->graphicsView->setRenderHint(QPainter::Antialiasing);
    // ui_->graphicsView->setCacheMode(QGraphicsView::CacheNone);
    // // ui_->graphicsView->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    // ui_->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    view_->setRenderHint(QPainter::Antialiasing);
    view_->setCacheMode(QGraphicsView::CacheNone);
    // view_->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    view_->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
}

void Dialog::SetupMapItem(int& rows, int& cols)
{
    rows_ = rows;
    cols_ = cols;
    mapItem_ = new MapItem(rows, cols);
    scene_->addItem(mapItem_);

#if DRAW_GRID
    // draw map grid
    QColor gridlinecolor = QColor(0, 0, 0, 50);
    for (int x = kXOrigin; x <= cols_; ++x) {
        scene_->addLine(x * kOneCellPx, 0,
                        x * kOneCellPx, rows_ * kOneCellPx,
                        QPen(gridlinecolor));
    }

    // Now add the horizontal lines, paint them green
    for (int y = kYOrigin; y <= rows_; ++y) {
        scene_->addLine(0,                  y * kOneCellPx,
                        cols_ * kOneCellPx, y * kOneCellPx,
                        QPen(gridlinecolor));
    }
#endif

    // Fit the view in the scene's bounding rect
    // view_->fitInView(scene_->itemsBoundingRect());
}

void Dialog::SetupRobotItem()
{
    robotItem_ = new RobotItem();
    scene_->addItem(robotItem_);
}

void Dialog::SetupTimer()
{
    // timeout() = every time the timer ticks
    // advance() = the scene has a slot called advance() that notifies each item
    // on the scene to advance
    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), scene_, SLOT(advance()));
}

void Dialog::StartTimer()
{
    timer_->start(kTimerTick_ms); // ticks every kTimerTick_ms milliseconds
}

void Dialog::Show()
{
    view_->show();
}

void Dialog::Destroy()
{

}

void Dialog::SetCurrentRobotPlan(cs::Plan& plan)
{
    robotItem_->SetCurrentPlan(plan);
    mapItem_->SetCurrentPlan(plan);
}

} // namespace vis
