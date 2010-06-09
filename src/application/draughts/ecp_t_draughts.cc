//#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp_t_draughts.h"
#include <stdio.h>
#include <unistd.h>
//#include <netdb.h>
//#include <netinet/in.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>

#define GRIPPER_OPENED 0.090
#define GRIPPER_CLOSED 0.057

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

const double Draughts::moves_table[32][8] = { { 0.000000, -0.124080, -1.766558,
		0.303766, -0.000520, 4.711508, -0.174239, 0.074000 }, { 0.000000,
		-0.030144, -1.787992, 0.302111, -0.000520, 4.711748, -0.079714,
		0.074000 }, { 0.000000, 0.070615, -1.793498, 0.298001, -0.001312,
		4.714027, 0.023445, 0.074000 }, { 0.000000, 0.178954, -1.779448,
		0.298321, -0.001312, 4.714147, 0.132673, 0.074000 }, { 0.000000,
		-0.168046, -1.668202, 0.313602, -0.000808, 4.715826, -0.216945,
		0.074000 }, { 0.000000, -0.079182, -1.695164, 0.313495, -0.000880,
		4.715946, -0.127858, 0.074000 }, { 0.000000, 0.026766, -1.696841,
		0.313442, -0.001600, 4.716426, -0.022969, 0.074000 },
		{ 0.000000, 0.117379, -1.693539, 0.313549, -0.001744, 4.715706,
				0.067989, 0.074000 }, { 0.000000, -0.115509, -1.595473,
				0.320616, -0.003040, 4.716786, -0.164626, 0.074000 }, {
				0.000000, -0.023788, -1.614115, 0.320616, -0.003040, 4.715466,
				-0.072428, 0.074000 }, { 0.000000, 0.071373, -1.606938,
				0.321687, -0.003040, 4.715946, 0.021085, 0.074000 }, {
				0.000000, 0.163735, -1.595685, 0.321687, -0.003328, 4.716066,
				0.113674, 0.074000 }, { 0.000000, -0.150553, -1.508203,
				0.318045, -0.003256, 4.715466, -0.204471, 0.074000 }, {
				0.000000, -0.069036, -1.520787, 0.320723, -0.003400, 4.714267,
				-0.121953, 0.074000 }, { 0.000000, 0.024958, -1.533875,
				0.321366, -0.003400, 4.713907, -0.027265, 0.074000 }, {
				0.000000, 0.112772, -1.522765, 0.320562, -0.003543, 4.713907,
				0.062084, 0.074000 }, { 0.000000, -0.100465, -1.432287,
				0.315796, -0.000161, 4.707875, -0.148357, 0.074000 }, {
				0.000000, -0.015042, -1.443553, 0.317563, 0.000271, 4.707875,
				-0.059900, 0.074000 }, { 0.000000, 0.064026, -1.443389,
				0.318098, 0.000559, 4.707995, 0.019269, 0.074000 }, { 0.000000,
				0.152015, -1.432232, 0.317135, 0.000919, 4.707995, 0.105474,
				0.074000 }, { 0.000000, -0.138949, -1.341688, 0.302912,
				-0.000449, 4.710034, -0.187039, 0.074000 }, { 0.000000,
				-0.057957, -1.360178, 0.308629, -0.000377, 4.710034, -0.107598,
				0.074000 }, { 0.000000, 0.023384, -1.367059, 0.310286,
				-0.000089, 4.709795, -0.026275, 0.074000 }, { 0.000000,
				0.105076, -1.360567, 0.309805, 0.000199, 4.709795, 0.055113,
				0.074000 }, { 0.000000, -0.095567, -1.274529, 0.294001,
				-0.000592, 4.709914, -0.145431, 0.074000 }, { 0.000000,
				-0.018016, -1.281652, 0.296134, -0.000305, 4.709555, -0.067382,
				0.074000 }, { 0.000000, 0.060119, -1.286032, 0.297254,
				-0.000089, 4.709555, 0.010765, 0.074000 }, { 0.000000,
				0.137554, -1.270760, 0.293148, 0.000127, 4.709914, 0.088412,
				0.074000 }, { 0.000000, -0.131427, -1.186187, 0.273826,
				-0.000736, 4.711474, -0.179655, 0.074000 }, { 0.000000,
				-0.057141, -1.204628, 0.278399, -0.000736, 4.711234, -0.105227,
				0.074000 }, { 0.000000, 0.019244, -1.207085, 0.279622,
				-0.000449, 4.711234, -0.028080, 0.074000 }, { 0.000000,
				0.095746, -1.201056, 0.278186, -0.000377, 4.710994, 0.048109,
				0.074000 } };

const double Draughts::wkings_table[8][8] = {//red, yellow kings
		{ 0.000000, -0.302915, -1.724556, 0.308415, 0.009988, 4.685923,
				-0.343790, 0.090 }, { 0.000000, -0.278075, -1.594786, 0.317884,
				0.010564, 4.686282, -0.318984, 0.090 }, { 0.000000, -0.257084,
				-1.446867, 0.314512, 0.011284, 4.686163, -0.299181, 0.090 }, {
				0.000000, -0.239008, -1.333692, 0.302271, 0.011787, 4.686882,
				-0.283673, 0.090 }, { 0.000000, -0.220466, -1.180818, 0.270107,
				0.011643, 4.687362, -0.262674, 0.090 }, { 0.000000, -0.201923,
				-1.388780, 0.309965, 0.011931, 4.684723, -0.232257, 0.090 }, {
				0.000000, -0.220757, -1.533555, 0.317831, 0.011140, 4.685323,
				-0.242055, 0.090 }, { 0.000000, -0.241049, -1.684678, 0.312318,
				0.010204, 4.687842, -0.264218, 0.090 }, };

const double Draughts::bkings_table[8][8] = {//blue, green kings
		{ 0.000000, 0.233065, -1.111134, 0.245453, 0.019489, 4.695999,
				0.208819, 0.090 }, { 0.000000, 0.242336, -1.232106, 0.277867,
				0.020209, 4.693960, 0.214659, 0.090 }, { 0.000000, 0.252423,
				-1.350054, 0.298215, 0.012219, 4.707155, 0.232309, 0.090 }, {
				0.000000, 0.267642, -1.477877, 0.314084, 0.012579, 4.707035,
				0.258887, 0.090 }, { 0.000000, 0.286418, -1.624080, 0.315743,
				0.012651, 4.708235, 0.285095, 0.090 }, { 0.000000, 0.212190,
				-1.448116, 0.312051, 0.012723, 4.708835, 0.189842, 0.090 }, {
				0.000000, 0.198720, -1.307385, 0.293414, 0.012651, 4.709675,
				0.174487, 0.090 }, { 0.000000, 0.185134, -1.184831, 0.266337,
				0.012651, 4.710154, 0.152780, 0.090 }, };

/*==============================Constructor==================================*/
//Constructors
Draughts::Draughts(lib::configurator &_config): task(_config){
	sensor_m[lib::SENSOR_CVFRADIA] = new fradia_sensor_board_and_draughts(this->config, "[vsp_fradia_sensor]");
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	ecp_m_robot = new irp6ot_m::robot(*this); //initialization of robot

	//sgen=new common::generator::smooth(*this, true);
	sgen2 = new common::generator::smooth(*this, true);
	befgen=new common::generator::bias_edp_force(*this);
	gagen=new common::generator::tff_gripper_approach (*this, 8);	//gripper approach constructor (task&, no_of_steps)
	sleepgen=new common::generator::sleep(*this);
	aitrans=new ecp_mp::transmitter::TRDraughtsAI(ecp_mp::transmitter::TRANSMITTER_DRAUGHTSAI, "[transmitter_draughts_ai]",*this);

	follower_vis = new generator::ecp_vis_ib_eih_follower_irp6ot(*this);	//follower servomechanism generator
	follower_vis->sensor_m = sensor_m;
}
;

/*============================Destructor=====================================*/
Draughts::~Draughts() {
	delete ecp_m_robot;
	//delete sgen;
	delete sgen2;
	delete befgen;
	delete gagen;
	delete sleepgen;
	delete aitrans;
	delete follower_vis;
}
;

/*=============================bPawn2bKing===================================*/
void Draughts::bPawn2bKing(int from, int to) {
	if (bkings > 8) {
		printf("there is no more kings available");
		return;
	}
	throwPawn(from);
	takeStaticPawn(bkings, BKING);
	bkings++;
	putStaticPawn(to);
}

/*=============================closeGripper==================================*/
void Draughts::closeGripper() {
	/*
	 sgen2->reset();
	 sgen2->set_relative();
	 sgen2->load_coordinates(lib::XYZ_ANGLE_AXIS,0,0,0,0,0,0,-0.033,0,true);
	 sgen2->Move();
	 */
}

/*===============================fradiaControl=================================*/
void Draughts::fradiaControl(DRAUGHTS_MODE dmode, char pawn_nr=-1){
	fradia_sensor_board_and_draughts * vsp_fr = dynamic_cast<fradia_sensor_board_and_draughts *> (sensor_m[lib::SENSOR_CVFRADIA]);

	draughts_control msg;
	msg.draughts_mode=dmode;
	msg.pawn_nr=pawn_nr;
	vsp_fr->set_initiate_message(msg);

	snooze(0.4);						//wait until signal is read
}

/*===========================gatAIMove=======================================*/
//1 get Black move - blue
//0 get White move - red
void Draughts::getAIMove(int player){
	vsp_fradia = dynamic_cast<fradia_sensor_board_and_draughts *> (sensor_m[lib::SENSOR_CVFRADIA]);

	vsp_fradia->get_reading();
	while(vsp_fradia->get_report() == lib::VSP_SENSOR_NOT_CONFIGURED){
			vsp_fradia->get_reading();
	}

	int port = config.value<int> ("ai_port", "[AI]");
	std::string node_name = config.value<std::string> ("ai_node_name", "[AI]");

	aitrans->AIconnect(node_name.c_str(), port);

	vsp_fradia->get_reading();
	printf("dane: \n");
	for (int i = 0; i < 32; i++) {
		aitrans->to_va.board[i]=vsp_fradia->get_reading_message().fields[i];
		printf("%d ",aitrans->to_va.board[i]);
	}
	aitrans->to_va.player=player;
	printf("\n");
	aitrans->t_write();
	aitrans->t_read();
	//printf("result %s ,%d\n",aitrans->from_va.draughts_ai.move,aitrans->from_va.draughts_ai.status);
	//strcpy(result,aitrans->from_va.draughts_ai.move);

	aitrans->AIdisconnect();
}

/*===============================getBoardStatus================================*/
BOARD_STATUS Draughts::getBoardStatus(){
	vsp_fradia = dynamic_cast<fradia_sensor_board_and_draughts *> (sensor_m[lib::SENSOR_CVFRADIA]);

	vsp_fradia->get_reading();
	while(vsp_fradia->get_report() == lib::VSP_SENSOR_NOT_CONFIGURED){
			vsp_fradia->get_reading();
	}

	vsp_fradia->get_reading();
	return vsp_fradia->get_reading_message().status;
}

/*============================goToInitialPos=================================*/
void Draughts::goToInitialPos() {
	sgen2->reset();
	sgen2->set_absolute();
	sgen2->load_coordinates(lib::ECP_JOINT, 0, -0.013, -1.442, -0.275, 0.045,
			4.720, -0.100, 0.090, true);
	sgen2->Move();
}

/*===============================goUp=======================================*/
void Draughts::goUp() {

	double
			v[MAX_SERVOS_NR] = { 0.20, 0.20, 0.01, 0.20, 0.20, 0.20, 0.002,
					0.20 };
	double a[MAX_SERVOS_NR] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.1 };
	sgen2->reset();
	sgen2->set_relative();
	sgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, v, a, 0, 0, -0.1, 0, 0, 0,
			0, 0, true); //move up
	sgen2->Move();

}

/*============================init_tdes======================================*/
//inicjacja struktury tdes - trajectory description
void Draughts::init_tdes(lib::ECP_POSE_SPECIFICATION pspec, int internode_no) {
	tdes.arm_type = pspec;
	tdes.interpolation_node_no = 1;
	tdes.internode_step_no = internode_no; //motion time
	tdes.value_in_step_no = internode_no - 2; //motion time-2 ??
}

/*=========================main_task_algorithm===============================*/
void Draughts::main_task_algorithm(void) {
	wkings = 0;
	bkings = 0;
	goToInitialPos();
	fradiaControl(DETECT_BOARD_STATE);
	uint8_t choice;
	choice = choose_option(
			"Do you want to play: 1 - Black(blue), or 2 - White(red)", 2);
	if (choice == lib::OPTION_ONE) {
		sr_ecp_msg->message("You will play black, while I will play white");
		printf("You will play black, while I will play white\n");
		while (true) {
			wait4move();
			if (makeAIMove(0) != AI_NORMAL_MOVE)
				break;
		}
	} else if (choice == lib::OPTION_TWO) {
		while (true) {
			sr_ecp_msg->message("You will play white, while I will play black");
			printf("You will play white, while I will play black\n");
			if (makeAIMove(1) != AI_NORMAL_MOVE)
				break;
			wait4move();
		}
	} else {
		sr_ecp_msg->message("Game canceled");
	}

	//takeDynamicPawn(31);

	fradiaControl(NONE);
	ecp_termination_notice();
}
;

/*===============================makeAIMove========================================*/
int Draughts::makeAIMove(int player) {
	char* move;
	char* board;
	int to;
	int movesnr;

	sleep(2);
	getAIMove(player);

	//printf("status: %d \nincoming: ",aitrans->from_va.draughts_ai.status);
	switch(aitrans->from_va.status){
		case AI_NORMAL_MOVE:
			printf("status: normal move\n");
			break;
		case AI_COMPUTER_WON:
			printf("status: computer won\n");
			break;
		case AI_HUMAN_WON:
			printf("status: human won\n");
			return AI_HUMAN_WON;
	}

	move=aitrans->from_va.move;
	board=aitrans->to_va.board;

	//count number of jumper fields

	for (movesnr = 0; movesnr < 25; movesnr++) {
		printf("%d ", move[movesnr]);
		if (move[movesnr] == -1) {
			break;
		}
	}

	movesnr--;
	//	printf("\nmovesnr: %d\n\n",movesnr);

	//chenge to dame
	//	printf("change: %d %d\n",move[movesnr],board[move[0]]);
	if ((move[movesnr] < 4 || move[movesnr] > 27) && board[move[0]] > 1) {
		if (board[move[0]] == 2) {
			wPawn2wKing(move[0], move[movesnr]);
			printf("change to whtie dame\n");
		} else if (board[move[0]] == 3) {
			printf("change to black dame\n");
			bPawn2bKing(move[0], move[movesnr]);
		}
	} else {
		//make move
		movePawn(move[0], move[movesnr]);
	}

	//remove jumped pawns
	if (movesnr > 1) {
		for (int i = 0; i < movesnr; i++)
			if (i % 2)
				throwPawn(move[i]);
	}
	goToInitialPos();

	if(aitrans->from_va.status==AI_COMPUTER_WON)
		return AI_COMPUTER_WON;

	return AI_NORMAL_MOVE;
}

/*=======================movePawn============================================*/
void Draughts::movePawn(int from, int to) {
	printf("moving pawn\n");
	//takeStaticPawn(from,PAWN);
	takeDynamicPawn(from);
	putStaticPawn(to);
}

/*===============================putStaticPawn===============================*/
void Draughts::putStaticPawn(int to) {
	printf("putting pawn\n");

	sgen2->reset();
	sgen2->set_absolute();
	sgen2->load_coordinates(lib::ECP_JOINT, moves_table[to][0],
			moves_table[to][1], moves_table[to][2], moves_table[to][3],
			moves_table[to][4], moves_table[to][5], moves_table[to][6],
			GRIPPER_CLOSED, true);
	sgen2->load_coordinates(lib::ECP_JOINT, moves_table[to][0],
			moves_table[to][1], moves_table[to][2], moves_table[to][3],
			moves_table[to][4], moves_table[to][5], moves_table[to][6],
			GRIPPER_OPENED, false);
	sgen2->Move();
	goUp();
}

/*=============================set_tdes======================================*/
void Draughts::set_tdes(double cor0, double cor1, double cor2, double cor3,
		double cor4, double cor5, double cor6) {
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	tdes.coordinate_delta[0] = cor0; // przyrost wspolrzednej X
	tdes.coordinate_delta[1] = cor1;// przyrost wspolrzednej Y
	tdes.coordinate_delta[2] = cor2; // przyrost wspolrzednej Z
	tdes.coordinate_delta[3] = cor3; // przyrost wspolrzednej FI
	tdes.coordinate_delta[4] = cor4; // przyrost wspolrzednej TETA
	tdes.coordinate_delta[5] = cor5; // przyrost wspolrzednej PSI
	tdes.coordinate_delta[6] = cor6; // przyrost wspolrzednej PSI
}

/*===============================snooze========================================*/
void Draughts::snooze(double time) {
	sleepgen->init_time(time);
	sleepgen->Move();
}

/*==============================takeDynamicPawn===============================*/
void Draughts::takeDynamicPawn(int from) {

	printf("taking pawn - servo\n");
	goToInitialPos();
	trackPawn(from);
	befgen->Move();

	//configuration of gripper approach configure(speed, time_period)
	gagen->configure(0.01, 300);
	gagen->Move();

	double
			v[MAX_SERVOS_NR] = { 0.20, 0.20, 0.01, 0.20, 0.20, 0.20, 0.002,
					0.20 };
	double a[MAX_SERVOS_NR] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.1 };

	sgen2->reset();
	sgen2->set_relative();
	sgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, v, a, 0, 0, -0.001, 0, 0,
			0, 0, 0, true); //go up to not touch board
	sgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, v, a, 0, 0, 0, 0, 0, 0,
			-0.033, 0, false); //close gripper
	sgen2->Move();
	goUp();

}

/*==============================takeStaticPawn===============================*/
void Draughts::takeStaticPawn(int from, int type) {

	printf("taking pawn\n");
	sgen2->reset();
	sgen2->set_absolute();

	if (type == PAWN) {
		sgen2->load_coordinates(lib::ECP_JOINT, moves_table[from][0],
				moves_table[from][1], moves_table[from][2],
				moves_table[from][3], moves_table[from][4],
				moves_table[from][5], moves_table[from][6],
				moves_table[from][7], true);
	} else if (type == WKING) {
		sgen2->load_coordinates(lib::ECP_JOINT, wkings_table[from][0],
				wkings_table[from][1], wkings_table[from][2],
				wkings_table[from][3], wkings_table[from][4],
				wkings_table[from][5], wkings_table[from][6],
				wkings_table[from][7], true);
	} else if (type == BKING) {
		sgen2->load_coordinates(lib::ECP_JOINT, bkings_table[from][0],
				bkings_table[from][1], bkings_table[from][2],
				bkings_table[from][3], bkings_table[from][4],
				bkings_table[from][5], bkings_table[from][6],
				bkings_table[from][7], true);
	}

	sgen2->Move();

	befgen->Move();

	//configuration of gripper approach configure(speed, time_period)
	gagen->configure(0.01, 300);
	gagen->Move();

	double
			v[MAX_SERVOS_NR] = { 0.20, 0.20, 0.01, 0.20, 0.20, 0.20, 0.002,
					0.20 };
	double a[MAX_SERVOS_NR] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.1 };

	sgen2->reset();
	sgen2->set_relative();
	sgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, v, a, 0, 0, -0.001, 0, 0,
			0, 0, 0, true); //go up to not touch board
	sgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, v, a, 0, 0, 0, 0, 0, 0,
			-0.033, 0, false); //close gripper
	sgen2->Move();
	goUp();

}

/*===============================throwPawn===================================*/
void Draughts::throwPawn(int from) {
	takeDynamicPawn(from);
	//takeStaticPawn(from,PAWN);
	sgen2->reset();
	sgen2->set_absolute();
	sgen2->load_coordinates(lib::ECP_JOINT, 0, -0.366, -1.184, 0.180, 0.01,
			4.704, -0.407, GRIPPER_CLOSED, true); //go beyond board
	sgen2->load_coordinates(lib::ECP_JOINT, 0, -0.366, -1.184, 0.180, 0.01,
			4.704, -0.407, GRIPPER_OPENED, false); //throw pawn
	sgen2->Move();
}

/*=======================trackPawn===========================================*/
void Draughts::trackPawn(char pawn_nr){
	fradiaControl(NONE);
	fradiaControl(TRACK_PAWN, pawn_nr);
	vsp_fradia = dynamic_cast<fradia_sensor_board_and_draughts *> (sensor_m[lib::SENSOR_CVFRADIA]);

	vsp_fradia->get_reading();
	while(vsp_fradia->get_report() == lib::VSP_SENSOR_NOT_CONFIGURED){
			vsp_fradia->get_reading();
	}

	sr_ecp_msg->message("Przed follower_vis");
	follower_vis->Move();
	sr_ecp_msg->message("Po follower_vis");
}

/*============================wait4move=======================================*/
void Draughts::wait4move(){
	BOARD_STATUS status;
	fradiaControl(STORE_BOARD);	//store board

	do{
		fradiaControl(NONE);			//do not analyze board for a while
		snooze(1);							//wait 2 seconds and..
		fradiaControl(CHECK_MOVE);
		status=getBoardStatus();			//check if state of the board changed
		switch(status){
			case STATE_CHANGED:
				printf("state changed\n");
				break;
			case STATE_UNCHANGED:
				printf("state unchanged\n");
				break;
			case BOARD_DETECTION_ERROR:
				printf("board detection error\n");
				break;
		}
	}while(status!=STATE_CHANGED);

	fradiaControl(DETECT_BOARD_STATE);
}

/*=============================wPawn2wKing===================================*/
void Draughts::wPawn2wKing(int from, int to) {
	if (wkings > 8) {
		printf("there is no more kings available");
		return;
	}

	throwPawn(from);
	takeStaticPawn(wkings, WKING);
	wkings++;
	putStaticPawn(to);
}
} //namespace task
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6ot_m::task::Draughts(_config);

}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

