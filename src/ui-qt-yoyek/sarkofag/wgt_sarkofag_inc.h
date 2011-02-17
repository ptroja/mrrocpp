#ifndef WGT_SARKOFAG_INC_H
#define WGT_SARKOFAG_INC_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_sarkofag_inc.h"
#include "../base/wgt_base.h"
#include <QTimer>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace sarkofag {
class UiRobot;
const std::string WGT_SARKOFAG_INC = "WGT_SARKOFAG_INC";
}
}
}

class wgt_sarkofag_inc : public wgt_base
{
    Q_OBJECT

public:
    wgt_sarkofag_inc(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::sarkofag::UiRobot& _robot, QWidget *parent = 0);
    ~wgt_sarkofag_inc();


private:
    Ui::wgt_sarkofag_incClass ui;
	mrrocpp::ui::sarkofag::UiRobot& robot;

private slots:




};

#endif // WGT_SARKOFAG_INC_H
