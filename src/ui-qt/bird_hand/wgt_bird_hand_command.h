#ifndef WGT_BIRD_HAND_COMMAND_H
#define WGT_BIRD_HAND_COMMAND_H

#include <QWidget>
#include "../base/wgt_base.h"


namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace bird_hand {
class UiRobot;
const std::string WGT_BIRD_HAND_COMMAND = "WGT_BIRD_HAND_COMMAND";
}
}
}

namespace Ui {
  class wgt_bird_hand_commandClass;
}

class wgt_bird_hand_command : public wgt_base
{
    Q_OBJECT

public:
    explicit wgt_bird_hand_command(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::bird_hand::UiRobot& _robot, QWidget *parent = 0);
    ~wgt_bird_hand_command();

private:
    Ui::wgt_bird_hand_commandClass *ui;
    mrrocpp::ui::bird_hand::UiRobot& robot;
};

#endif // WGT_BIRD_HAND_COMMAND_H
