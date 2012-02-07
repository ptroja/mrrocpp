#ifndef WGT_PLAN_H
#define WGT_PLAN_H

#include <iostream>

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>

#include "../wgt_base.h"
#include "../interface.h"

#include "base/lib/exception.h"

#include "ui_wgt_plan.h"

class wgt_plan : public wgt_base
{
Q_OBJECT

public:
	wgt_plan(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_plan();

	void hideEvent(QHideEvent * event);
	void my_open(bool set_on_top = false);

	REGISTER_FATAL_ERROR(plan_item_out_of_range, "plan item value is out of range");

private:
	Ui::wgt_planClass* ui;

	//! Deactivate widget into idle mode
	void reply();

	//! Reload inputs with original request
	void reload();

	//! Check if value is within widget limits.
	template<class WIDGET_T, class VALUE_T>
	void checkInputWidgetLimits(const WIDGET_T & widget, const VALUE_T value)
	{
		if ((value < widget.minimum()) ||(value > widget.maximum())) {
			std::stringstream msg;

			msg << "Input widget value " << value
					<< " out of range <" << widget.minimum()
					<< ".."
					<< widget.maximum()
					<< ">";

			std::cerr << msg.str() << std::endl;

			interface.ui_msg->message(lib::NON_FATAL_ERROR, msg.str());
			BOOST_THROW_EXCEPTION(plan_item_out_of_range());
		}
	}

private slots:

	void on_pushButton_prev_clicked();
	void on_pushButton_next_clicked();
	void on_pushButton_exec_clicked();
	void on_pushButton_save_clicked();
	void on_pushButton_reload_clicked();
	void on_pin_input_valueChanged(int i);

};

#endif // WGT_PLAN_H
