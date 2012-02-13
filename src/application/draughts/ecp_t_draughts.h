#if !defined(_ECP_T_TB_IRP6OT_H)
#define _ECP_T_TB_IRP6OT_H

#include "base/ecp/ecp_task.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"

#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"
#include "generator/ecp/tff_gripper_approach/ecp_g_tff_gripper_approach.h"
#include "base/lib/com_buf.h"
#include "subtask/ecp_st_go.h"
#include "generator/ecp/sleep/ecp_g_sleep.h"
#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"
#include "ecp_mp_tr_draughtsAI.h"
#include "ecp_g_vis_ib_eih_follower_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

/*commands from mrrocpp used in Draughts task*/
typedef enum
{
	TRACK_PAWN, Z_TRACKER, DETECT_BOARD_STATE, NONE, STORE_BOARD, CHECK_MOVE
} DRAUGHTS_MODE;

/*information returned to mrrocpp used in Draughts task*/
typedef enum
{
	STATE_CHANGED, STATE_UNCHANGED, STATE_OK, BOARD_DETECTION_ERROR
} BOARD_STATUS;

/*
 * Structure for storing pawn coordinates from cvFraDIA
 * \author tbem
 */
typedef struct
{
	char fields[32];
	BOARD_STATUS status;
} board;

//structure for controlling fraDIA form mrrocpp
typedef struct
{
	DRAUGHTS_MODE draughts_mode;
	char pawn_nr;
} draughts_control;

const int WKING = 0, BKING = 1;
const int WMAN = 2, BMAN = 3;
const int EMPTY = 4, BORDER = 5;
const int PAWN = 6;

const int AI_NORMAL_MOVE = 0;
const int AI_COMPUTER_WON = 1;
const int AI_HUMAN_WON = 2;

typedef ecp_mp::sensor::fradia_sensor <lib::empty_t, board, draughts_control> fradia_sensor_board_and_draughts;

class Draughts : public common::task::task
{

private:
	fradia_sensor_board_and_draughts *vsp_fradia; //Virtual sensor
	//common::generator::smooth* sgen;				//smooth movement generator
	//common::generator::smooth* sgen2; //smooth movement generator
	common::generator::bias_edp_force* befgen; //calibration of force
	common::generator::tff_gripper_approach* gagen; //gripper approach with force control
	lib::trajectory_description tdes; //trajectory description from com_buf.h
	common::generator::sleep* sleepgen; //sleep generator
	ecp_mp::transmitter::TRDraughtsAI *aitrans; //AI transmiter
	generator::ecp_vis_ib_eih_follower_irp6ot* follower_vis; //Follower servomechanism

	static const double moves_table[32][8];
	static const double bkings_table[8][8];
	static const double wkings_table[8][8];
	int bkings;
	int wkings;
public:
	Draughts(lib::configurator &_config);
	~Draughts();
	void bPawn2bKing(int from, int to);
	void closeGripper();
	void fradiaControl(DRAUGHTS_MODE, char);
	void getAIMove(int player);
	BOARD_STATUS getBoardStatus();
	void goToInitialPos();
	void goUp();
	void init_tdes(lib::ECP_POSE_SPECIFICATION, int);
	void main_task_algorithm(void);
	int makeAIMove(int player);
	void movePawn(int from, int to);
	void putStaticPawn(int to);
	void set_tdes(double, double, double, double, double, double, double);
	void snooze(double);
	void takeDynamicPawn(int from);
	void takeStaticPawn(int from, int type);
	void throwPawn(int from);
	void trackPawn(char);
	void wait4move();
	void wPawn2wKing(int from, int to);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

