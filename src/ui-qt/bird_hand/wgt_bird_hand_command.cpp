#include "wgt_bird_hand_command.h"
#include "ui_wgt_bird_hand_command.h"

wgt_bird_hand_command::wgt_bird_hand_command(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::bird_hand::UiRobot& _robot, QWidget *parent) :
    wgt_base("Spkm incremental motion", _interface, parent),
    ui(new Ui::wgt_bird_hand_commandClass),
    robot(_robot)

{
    ui->setupUi(this);
}

wgt_bird_hand_command::~wgt_bird_hand_command()
{
    delete ui;
}
