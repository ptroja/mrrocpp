#ifndef WND_PROCESS_CONTROL_H
#define WND_PROCESS_CONTROL_H

#include <QtGui/QMainWindow>
#include "ui_wnd_process_control.h"

class wnd_process_control : public QMainWindow
{
    Q_OBJECT

public:
    wnd_process_control(QWidget *parent = 0);
    ~wnd_process_control();

private:
    Ui::wnd_process_controlClass ui;
};

#endif // WND_PROCESS_CONTROL_H
