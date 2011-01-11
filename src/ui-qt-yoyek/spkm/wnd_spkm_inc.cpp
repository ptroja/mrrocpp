#include "wnd_spkm_inc.h"
#include "../interface.h"

wnd_spkm_inc::wnd_spkm_inc(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QMainWindow(parent), interface(_interface)
{
	ui.setupUi(this);
}

wnd_spkm_inc::~wnd_spkm_inc()
{

}
