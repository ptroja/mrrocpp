#include <QTextCharFormat>
#include <QBrush>
#include <QColor>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "interface.h"

MainWindow::MainWindow(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QMainWindow(parent), ui(new Ui::MainWindow), interface(_interface)
{
	ui->setupUi(this);

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

	ui->plainTextEdit_sr->appendPlainText(interface.sr_attach_point.c_str());
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
