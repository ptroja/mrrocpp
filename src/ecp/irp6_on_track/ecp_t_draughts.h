#if !defined(_ECP_T_TB_IRP6OT_H)
#define _ECP_T_TB_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "lib/com_buf.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_g_sleep.h"
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp_mp/ecp_mp_tr_draughtsAI.h"
#include "ecp/irp6_on_track/ecp_vis_ib_eih_planar_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

const int WKING =0, BKING=1;
const int WMAN  =2, BMAN=3;
const int EMPTY=4,  BORDER=5;
const int PAWN=6;

const int AI_NORMAL_MOVE=0;
const int AI_COMPUTER_WON=1;
const int AI_HUMAN_WON=2;

class Draughts: public common::task::task{

	private:
		lib::sensor *vsp_fradia;						//Virtual sensor
		common::generator::smooth* sgen;				//smooth movement generator
		common::generator::bias_edp_force* befgen;		//calibration of force
		common::generator::tff_gripper_approach* gagen;	//gripper approach with force control
		common::generator::linear *lgen;				//linear generator
		lib::trajectory_description tdes;				//trajectory description from com_buf.h
		common::task::ecp_sub_task_gripper_opening* go_st;		//sub_task_gripper_opening
		common::generator::sleep* sleepgen;				//sleep generator
		ecp_mp::transmitter::TRDraughtsAI *aitrans;		//AI transmiter
		ecp_vis_ib_eih_planar_irp6ot* planar_vis;	//Planar servomechanism.

		static const double moves_table[32][8];
		static const double bkings_table[4][8];
		static const double wkings_table[4][8];
		int bkings;
		int wkings;
	public:
		Draughts(lib::configurator &_config);
		~Draughts();
		void set_tdes(double, double, double, double, double, double, double);
		void init_tdes(lib::POSE_SPECIFICATION, int);
		void task_initialization(void);
		void main_task_algorithm(void);
		int AIConnect(const char*,unsigned short int);
		void test();
		void getAIMove(int player);
		void movePawn(int from, int to);
		void trackPawn();
		void takeStaticPawn(int from, int type);
		void putStaticPawn(int to);
		void throwPawn(int from);
		void closeGripper();
		int makeAIMove(int player);
		void bPawn2bKing(int from, int to);
		void wPawn2wKing(int from, int to);
		void fradiaControl(lib::DRAUGHTS_MODE);
		void wait4move();
		void snooze(double);
		lib::BOARD_STATUS getBoardStatus();
		void goToInitialPos();
		void goUp();
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

