#ifndef WGTRELATIVEBASE_H_
#define WGTRELATIVEBASE_H_

#include "wgt_base.h"

class WgtRelativeBase : public wgt_base
{
	Q_OBJECT

public:
	WgtRelativeBase(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent =
			0);
	virtual ~WgtRelativeBase();

protected:
	void synchro_depended_widgets_disable(bool set_disabled);
	void setup_ui(QGridLayout *layout);
	const static int angle_axis_number;

private:
	void create_buttons();
	virtual void move_it()
	{
	}
	virtual void init()
	{
	}

	const static int desired_pos_column = 1;
	const static int inc_move_column = 9;

	QPushButton *l_button;
	QPushButton *r_button;

public slots:
	virtual void inc_move_left_button_clicked(int button);
	virtual void inc_move_right_button_clicked(int button);
	void r_button_clicked();
	void l_button_clicked();
	void init_and_copy_slot();
};

#endif /* WGTRELATIVEBASE_H_ */
