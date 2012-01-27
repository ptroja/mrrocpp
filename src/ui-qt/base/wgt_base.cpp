#include "wgt_base.h"
#include "interface.h"
#include "mainwindow.h"
#include "ui_robot.h"

#include <QMainWindow>
#include <QLocale>

wgt_base::wgt_base(const QString & _widget_label, mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
		QWidget(parent), widget_label(_widget_label), interface(_interface)
{
	dwgt = new QDockWidget((QMainWindow *) interface.get_main_window());
	//dwgt_pc->setAllowedAreas(Qt::TopDockWidgetArea);
	dwgt->setWindowTitle(widget_label);

	//vl = new QVBoxLayout();
	//dwgt->setLayout(vl);

	//	vl->addWidget(this);
	dwgt->setWidget(this);
	dwgt->hide();
	//dwgt->setFloating(true);
	interface.get_main_window()->addDockWidget(Qt::LeftDockWidgetArea, dwgt);
}

wgt_base::wgt_base(const QString & _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
		QWidget(parent), widget_label(_widget_label), interface(_interface), robot(robo)
{
	dwgt = new QDockWidget((QMainWindow *) interface.get_main_window());
	//dwgt_pc->setAllowedAreas(Qt::TopDockWidgetArea);
	dwgt->setWindowTitle(widget_label);

	//vl = new QVBoxLayout();
	//dwgt->setLayout(vl);

	//	vl->addWidget(this);
	dwgt->setWidget(this);
	dwgt->hide();
	//dwgt->setFloating(true);
	interface.get_main_window()->addDockWidget(Qt::LeftDockWidgetArea, dwgt);

	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(init_and_copy_signal()), this, SLOT(init_and_copy_slot()), Qt::QueuedConnection);
}

void wgt_base::my_open(bool set_on_top)
{
	if (!dwgt->isVisible() && interface.wgt_pc->dwgt != dwgt)
		interface.get_main_window()->tabifyDockWidget(interface.wgt_pc->dwgt, dwgt);

	dwgt->show();

	if (set_on_top)
		dwgt->raise();

	init_and_copy_slot();

}

void wgt_base::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_base::synchro_depended_init_slot()
{
	try {
		if (robot->state.edp.pid != -1) {
			if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
				synchro_depended_widgets_disable(false);
			else
				synchro_depended_widgets_disable(true); // Wygaszanie elementow przy niezsynchronizowanym robocie
		}
	}
	CATCH_SECTION_UI_PTR
}

void wgt_base::init_and_copy()
{
	emit init_and_copy_signal();
}

void wgt_base::get_desired_position()
{
	if (robot->state.edp.pid != -1) {
		if (robot->state.edp.is_synchronised)
			for (int i = 0; i < rows_number; i++)
				robot->desired_pos[i] = desired_pos_spin_boxes[i]->value();
		else
			for (int i = 0; i < rows_number; i++)
				robot->desired_pos[i] = 0.0;
	}
}

void wgt_base::setup_ui(QGridLayout *layout, int _rows_number)
{
	rows_number = _rows_number;
	gridLayout = layout;
}

void wgt_base::create_buttons_and_spin_boxes(int desiredPosColumn, int incMoveColumn, int spinBoxesCount)
{

	rows_number = spinBoxesCount;

	QSignalMapper *left_signal_mapper = new QSignalMapper(this);
	QSignalMapper *right_signal_mapper = new QSignalMapper(this);
	QPushButton *button = 0l;

	connect(left_signal_mapper, SIGNAL(mapped(int)), this, SLOT(inc_move_left_button_clicked(int)));
	connect(right_signal_mapper, SIGNAL(mapped(int)), this, SLOT(inc_move_right_button_clicked(int)));

	for (int i = 0; i < spinBoxesCount; i++) {
		button = create_button_to_vector(inc_move_left_buttons, QString("<"));
		add_incremental_move_button(button, i, incMoveColumn);
		connect_to_signal_mapper(button, i, left_signal_mapper);

		button = create_button_to_vector(inc_move_right_buttons, QString(">"));
		add_incremental_move_button(button, i, incMoveColumn + 1);
		connect_to_signal_mapper(button, i, right_signal_mapper);
	}

	create_spin_boxes(desiredPosColumn, spinBoxesCount);
}

void wgt_base::create_spin_boxes(int desiredPosColumn, int spinBoxesCount)
{
	QDoubleSpinBox *spinBox = 0l;
	for (int i = 0; i < spinBoxesCount; i++) {
		spinBox = create_spin_box_to_vector(desired_pos_spin_boxes);
		add_desired_position_spin_box(spinBox, i, desiredPosColumn);
		desired_pos_spin_boxes[i]->setValue(robot->desired_pos[i]);
	}
}

void wgt_base::connect_to_signal_mapper(QPushButton *button, int i, QSignalMapper *signalMapper)
{
	signalMapper->setMapping(button, i);
	connect(button, SIGNAL(clicked()), signalMapper, SLOT(map()));
}

QPushButton* wgt_base::create_button_to_vector(QVector <QPushButton *> &buttons, QString label)
{
	QPushButton *button;
	button = new QPushButton(this);
	button->setMaximumSize(QSize(20, 20));
	button->setIconSize(QSize(12, 12));
	button->setText(label);
	buttons.append(button);
	return button;
}

QDoubleSpinBox* wgt_base::create_spin_box_to_vector(QVector <QDoubleSpinBox*> &spin_boxes)
{
	QDoubleSpinBox *spin_box;
	spin_box = new QDoubleSpinBox(this);
	spin_box->setDecimals(3);
	spin_box->setMinimum(-1000);
	spin_box->setMaximum(1000);
	spin_box->setSingleStep(0.5);
	spin_boxes.append(spin_box);
	return spin_box;
}

void wgt_base::add_incremental_move_button(QPushButton *button, int row, int column)
{
	gridLayout->addWidget(button, row + 1, column, 1, 1);
}

void wgt_base::add_desired_position_spin_box(QDoubleSpinBox *spin_box, int row, int column)
{
	spin_box->setValue(robot->desired_pos[row]);
	gridLayout->addWidget(spin_box, row + 1, column, 1, 2);
}

QPushButton* wgt_base::add_button(QString label, int x, int y, int rowSpan, int columnSpan)
{
	QPushButton *button = new QPushButton(label, this);
	QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	sizePolicy.setHorizontalStretch(0);
	sizePolicy.setVerticalStretch(0);
	sizePolicy.setHeightForWidth(button->sizePolicy().hasHeightForWidth());
	button->setSizePolicy(sizePolicy);
	button->setMaximumSize(QSize(60, 16777215));
	button->setIconSize(QSize(12, 12));
	gridLayout->addWidget(button, x, y, rowSpan, columnSpan);
	return button;
}

void wgt_base::my_close()
{
	dwgt->hide();
}
