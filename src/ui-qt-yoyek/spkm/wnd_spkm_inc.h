#ifndef WND_SPKM_INC_H
#define WND_SPKM_INC_H

#include <QtGui/QWidget>
#include "ui_wnd_spkm_inc.h"

class wnd_spkm_inc : public QWidget
{
    Q_OBJECT

public:
    wnd_spkm_inc(QWidget *parent = 0);
    ~wnd_spkm_inc();

private:
    Ui::wnd_spkm_incClass ui;
};

#endif // WND_SPKM_INC_H
