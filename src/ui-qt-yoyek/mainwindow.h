#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();
	void init();

	std::string sr_attach_point;
	bool is_sr_thread_loaded;

private:
	Ui::MainWindow *ui;

private slots:
	void on_pushButton_l1_clicked();
	void on_pushButton_l2_clicked();
	void on_actionClear_Console_triggered();

};

#endif // MAINWINDOW_H
