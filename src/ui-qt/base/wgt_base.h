#ifndef WGT_BASE_H
#define WGT_BASE_H

#include <QtGui/QWidget>
//#include <QVBoxLayout>
#include <QDockWidget>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QSignalMapper>
#include <iostream>
#include "ui_ecp.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
}
}
}

class wgt_base : public QWidget //TODO: zmienić dziedziczenie na QDockWidget!
{
	Q_OBJECT

public:
	wgt_base(const QString & _widget_label, mrrocpp::ui::common::Interface & _interface, QWidget *parent = 0);
	wgt_base(const QString & _widget_label, mrrocpp::ui::common::Interface & _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent);

	virtual void my_open(bool set_on_top = false);
	void my_close();

	QString widget_label;
	QDockWidget* dwgt;

	virtual void synchro_depended_init();
	virtual void init_and_copy();
	virtual void synchro_depended_widgets_disable(bool set_disabled)
	{
	}

	typedef void (wgt_base::*my_open_ptr)(bool set_on_top);

protected:
	mrrocpp::ui::common::Interface& interface;
	mrrocpp::ui::common::UiRobot *robot;

	int rows_number;

	QVector <QPushButton *> inc_move_left_buttons;
	QVector <QPushButton *> inc_move_right_buttons;
	QVector <QDoubleSpinBox*> desired_pos_spin_boxes;

	QGridLayout *gridLayout;

	void add_incremental_move_button(QPushButton *button, int row, int column);
	void add_desired_position_spin_box(QDoubleSpinBox *spin_box, int row, int column);

	virtual void create_buttons_and_spin_boxes(int desiredPosColumn, int incMoveColumn, int spinBoxesCount);

	void create_spin_boxes(int desiredPosColumn, int spinBoxesCount);

	virtual void setup_ui(QGridLayout *layout, int _rows_number);
	virtual void get_desired_position();

	QDoubleSpinBox* create_spin_box_to_vector(QVector <QDoubleSpinBox*> &spin_boxes);
	QPushButton* create_button_to_vector(QVector <QPushButton *> &buttons, QString label);
	QPushButton* add_button(QString label, int x, int y, int rowSpan, int columnSpan);

	signals:
	void synchro_depended_init_signal();
	void init_and_copy_signal();

public slots:
	virtual void inc_move_left_button_clicked(int button)
	{
	}
	virtual void inc_move_right_button_clicked(int button)
	{
	}
	virtual void init_and_copy_slot()
	{
	}

private slots:
	void synchro_depended_init_slot();

private:
	void connect_to_signal_mapper(QPushButton *button, int i, QSignalMapper *signalMapper);
};

#endif // WGT_PROCESS_CONTROL_H
