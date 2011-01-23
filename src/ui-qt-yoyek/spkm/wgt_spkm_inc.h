#ifndef WGT_SPKM_INC_H
#define WGT_SPKM_INC_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_spkm_inc.h"
#include "../wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace spkm {
class UiRobot;
const std::string WGT_SPKM_INC = "WGT_SPKM_INC";
}
}
}

class wgt_spkm_inc : public wgt_base
{
Q_OBJECT

public:
	wgt_spkm_inc(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::spkm::UiRobot& _robot, QWidget *parent = 0);
	~wgt_spkm_inc();

private:
	Ui::wgt_spkm_incClass ui;
	mrrocpp::ui::spkm::UiRobot& robot;

	int init();
	int import();
	int exporto();
	int copy();
	int motion(/* TR PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo*/);

	int set_single_axis(int axis/* TR, PtWidget_t *ABW_current, PtWidget_t *ABW_position, PtWidget_t *ABW_thumb*/);

};

#endif // WGT_SPKM_INC_H
