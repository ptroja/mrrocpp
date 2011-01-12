#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

namespace Ui {
class MainWindow;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
	explicit MainWindow(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~MainWindow();

private:
	Ui::MainWindow *ui;
	mrrocpp::ui::common::Interface& interface;
	QTimer *timer;

private slots:
	void on_pushButton_l1_clicked();
	void on_pushButton_l2_clicked();
	void on_timer_slot();

	// menus

	// file menu
	void on_actionQuit_triggered();

	// robot menu

	// spkm menu
	void on_actionspkm_EDP_load_triggered();
	void on_actionspkm_EDP_unload_triggered();
	void on_actionspkm_Motors_triggered();

	// special menu
	void on_actionClear_Console_triggered();
	void on_actionUnload_All_triggered();
	void on_actionSlay_All_triggered();

};

#endif // MAINWINDOW_H
