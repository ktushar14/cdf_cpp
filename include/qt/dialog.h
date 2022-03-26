#ifndef DIALOG_H
#define DIALOG_H

#include "covsearch/types.hpp"
#include "qt/map_item.hpp"
#include "qt/robot_item.hpp"

#include <QDebug>
#include <QDialog>
#include <QGraphicsView>
#include <QMainWindow>
#include <QGraphicsItem>
#include <QGraphicsRectItem>
#include <QtCore>
#include <QtGui>

#include <memory>

namespace Ui {
class Dialog;
}

namespace vis {

// class Dialog : public QDialog {
class Dialog : public QMainWindow {
    Q_OBJECT

public:
    explicit Dialog(QWidget* parent = 0);
    ~Dialog();

    void SetCoverageMap(cs::BooleanMap& coverageMap);

    void SetupGraphicsScene(int& rows, int& cols);
    void SetupMapItem(int& rows, int& cols);
    void SetupRobotItem();
    void SetupTimer();
    void StartTimer();
    void Show();
    void Destroy();

    void SetCurrentRobotPlan(cs::Plan& plan);

    // Ui::Dialog* ui_;
    std::unique_ptr<QGraphicsView> view_;
    QGraphicsScene* scene_;
    QTimer* timer_;

    // A cell indicating the origin
    QGraphicsRectItem* originCell_;

    MapItem* mapItem_;
    RobotItem* robotItem_;

    int rows_, cols_;
};

} // namespace vis

#endif // DIALOG_H
