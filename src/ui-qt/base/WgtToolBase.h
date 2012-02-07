#ifndef WGTTOOLBASE_H_
#define WGTTOOLBASE_H_

#include "wgt_base.h"
#include "ui_wgt_tool_template.h"

class WgtToolBase : public wgt_base
{
Q_OBJECT

public:
	WgtToolBase(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent =
			0);
	virtual ~WgtToolBase();

	Ui::wgt_tool_template ui;

protected:
	QVector <QDoubleSpinBox*> current_pos_spin_boxes;
	void synchro_depended_widgets_disable(bool set_disabled);
	void setup_ui(QGridLayout *layout);
	const static int angle_axis_number;
	void create_buttons_and_spin_boxes();

private:

	void create_buttons();
	virtual void move_it()
	{
	}
	virtual void init()
	{
	}
	virtual void add_current_position_spin_box(QDoubleSpinBox *spin_box, int row);
	void copy();
	const static int desired_pos_column = 5;
	const static int inc_move_column = 9;

	QPushButton *read_button;
	QPushButton *set_button;
	QPushButton *copy_button;

public slots:

	virtual void inc_move_left_button_clicked(int button);
	virtual void inc_move_right_button_clicked(int button);
	void init_and_copy_slot();
	void read_button_clicked();
	void copy_button_clicked();
	void execute_button_clicked();
};

#endif /* WGTTOOLBASE_H_ */
