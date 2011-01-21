#ifndef WGT_SPKM_INC_H
#define WGT_SPKM_INC_H

#include <QtGui/QWidget>
#include "ui_wgt_spkm_inc.h"

class wgt_spkm_inc : public QWidget
{
    Q_OBJECT

public:
    wgt_spkm_inc(QWidget *parent = 0);
    ~wgt_spkm_inc();

private:
    Ui::wgt_spkm_incClass ui;
};

#endif // WGT_SPKM_INC_H
