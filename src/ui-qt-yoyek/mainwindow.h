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

private:
	Ui::MainWindow *ui;

private slots:
	void on_pushButton_l1_clicked();
	void on_pushButton_l2_clicked();
	void on_actionClear_Console_triggered();

};

#endif // MAINWINDOW_H
