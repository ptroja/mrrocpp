#include <QTextCharFormat>
#include <QBrush>
#include <QColor>

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);

}

void MainWindow::init()
{
	is_sr_thread_loaded = false;
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::on_pushButton_l1_clicked()
{
	QTextCharFormat format;
	format.setFontItalic(true);

	format.setForeground(Qt::red);
	ui->plainTextEdit_sr->setCurrentCharFormat(format);

	ui->plainTextEdit_sr->appendPlainText("l1");
}

void MainWindow::on_pushButton_l2_clicked()
{
	QTextCharFormat format;
	format.setFontItalic(false);

	format.setForeground(Qt::blue);
	ui->plainTextEdit_sr->setCurrentCharFormat(format);

	ui->plainTextEdit_sr->appendPlainText("l2");
}

void MainWindow::on_actionClear_Console_triggered()
{
	ui->plainTextEdit_sr->clear();
}
