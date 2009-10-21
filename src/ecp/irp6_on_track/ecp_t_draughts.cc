//#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_t_draughts.h"
#include <stdio.h>
#include <unistd.h>
//#include <netdb.h>
//#include <netinet/in.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>

#define GRIPPER_OPENED 800.0
#define GRIPPER_CLOSED 1000.0


using namespace std;

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

const double Draughts::moves_table[32][8]= {
	{0.000000,   6.762255, -40.591957, 49.588738, 151.072834, -11.583006, 719.987063, -1800},
	{0.000000,  -9.507694, -43.171564, 49.100455, 150.879363, -11.813328, 749.800777, -1800},
	{0.000000, -25.878985, -42.830687, 49.045178, 150.676680, -12.062076, 779.796704, -1800},
	{0.000000, -41.734354, -38.905999, 49.515035, 150.501635, -12.320037, 808.828161, -1800},
	{0.000000,  13.303401, -22.838734, 51.385250, 151.174176, -11.693561, 708.121267, -1800},
	{0.000000,  -1.575403, -26.542312, 51.035160, 150.989918, -11.850180, 735.440557, -1800},
	{0.000000, -17.182024, -27.841328, 50.823264, 150.814873, -12.034438, 763.991351, -1800},
	{0.000000, -32.696517, -25.713153, 50.878541, 150.658254, -12.237121, 792.388207, -1800},
	{0.000000,   4.836763, -10.576388, 51.901171, 151.082047, -11.776477, 723.694142, -1800},
	{0.000000,  -9.968338, -12.722990, 51.772191, 150.943853, -11.951522, 750.746396, -1800},
	{0.000000, -24.865568, -12.446603, 51.707701, 150.796447, -12.126566, 778.015421, -1800},
	{0.000000, -39.265302,  -9.019411, 51.744552, 150.621403, -12.301611, 804.329401, -1800},
	{0.000000,  10.990968,   5.887031, 52.011726, 151.128111, -11.840967, 712.177063, -1800},
	{0.000000,  -2.773077,   2.791503, 52.057790, 151.008344, -11.933096, 737.419760, -1800},
	{0.000000, -17.006980,   1.556977, 51.744552, 150.860938, -12.062076, 763.535820, -1800},
	{0.000000, -31.019773,   3.325850, 51.606359, 150.722744, -12.209482, 789.158650, -1800},
	{0.000000,   3.288999,  17.836139, 51.182567, 151.063621, -11.923883, 726.329938, -1800},
	{0.000000, -10.253937,  15.643473, 51.274695, 150.925428, -12.071289, 751.054273, -1800},
	{0.000000, -23.815299,  16.187033, 51.200992, 150.778022, -12.218695, 775.907412, -1800},
	{0.000000, -36.934443,  18.610021, 50.961457, 150.649041, -12.375314, 799.830640, -1800},
	{0.000000,   8.825941,  32.807072, 49.183371, 151.100472, -11.868606, 716.151178, -1800},
	{0.000000,  -3.906262,  30.089272, 49.616377, 150.953066, -11.997586, 739.380114, -1800},
	{0.000000, -16.730593,  29.131132, 49.699293, 150.824086, -12.135779, 762.891794, -1800},
	{0.000000, -29.785246,  30.734173, 49.386055, 150.704319, -12.283185, 786.683075, -1800},
	{0.000000,   1.814938,  43.899381, 47.064408, 151.035982, -11.942309, 728.994009, -1800},
	{0.000000, -10.751433,  42.001528, 47.396072, 150.897789, -12.071289, 751.943343, -1800},
	{0.000000, -23.262526,  42.517449, 47.174963, 150.778022, -12.200269, 774.845554, -1800},
	{0.000000, -35.322188,  44.958863, 46.631403, 150.676680, -12.301611, 796.952941, -1800},
	{0.000000,   6.706978,  57.635788, 42.434533, 151.155750, -11.850180, 720.323213, -1800},
	{0.000000,  -4.919679,  55.332567, 44.070222, 151.035982, -11.951522, 741.607503, -1800},
	{0.000000, -16.666103,  54.835072, 44.208415, 150.897789, -12.098928, 762.872944, -1800},
	{0.000000, -28.900810,  55.986682, 42.895177, 150.741170, -12.246334, 785.193960, -1800},
	};

const double Draughts::wkings_table[8][8]={
	{0.000000, 34.345623, -33.967895, 47.400276, 153.219435, -9.970752, 669.621049, -1799.936241},
	{0.000000, 24.764226, -26.708144, 47.971475, 152.768004, -10.376119, 687.257950, -1800.034416},
	{0.000000, 30.743386, -11.073884, 48.901976, 153.025965, -10.127371, 676.051889, -1799.985328},
	{0.000000, 21.585782, -0.101342, 48.800634, 152.574534, -10.505099, 693.066755, -1799.936241},
	{0.000000, 27.279343, 14.344457, 48.155733, 152.860133, -10.274777, 682.445031, -1799.985328},
	{0.000000, 19.245710, 21.364673, 47.308147, 152.473192, -10.624867, 697.333038, -1800.034416},
	{0.000000, 24.681310, 34.843119, 45.539274, 152.721940, -10.394545, 687.220251, -1799.936241},
	{0.000000, 22.120129, 58.004303, 40.177377, 152.602172, -10.505099, 692.052021, -1799.936241}
};

const double Draughts::bkings_table[8][8]={
	{0.000000, -49.648219, 67.908151, 35.561723, 148.917020, -14.254742, 823.408293, -1799.985328},
	{0.000000, -44.221832, 58.566289, 38.648039, 149.193406, -13.950717, 813.521701, -1799.936241},
	{0.000000, -51.785608, 48.201797, 41.190794, 148.797252, -14.374509, 827.385549, -1799.985328},
	{0.000000, -44.829882, 34.419326, 44.111277, 149.147342, -13.987568, 814.611833, -1800.034416},
	{0.000000, -52.550277, 27.150362, 45.087843, 148.769613, -14.420574, 828.818115, -1799.985328},
	{0.000000, -46.911994, 13.450807, 46.700097, 149.055213, -14.088910, 818.466568, -1799.985328},
	{0.000000, -55.258864,  6.080502, 47.004122, 148.640633, -14.577193, 833.712717, -1799.936241},
	{0.000000, -58.271477, -15.901434, 46.985697, 148.493227, -14.752238, 839.311035, -1800.034416}
};



/*==============================Constructor==================================*/
//Constructors
Draughts::Draughts(lib::configurator &_config): task(_config){
	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA,"[vsp_cvfradia]", *this,	sizeof(lib::sensor_image_t::sensor_union_t::fradia_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	ecp_m_robot=new robot(*this);				//initialization of robot

	sgen=new common::generator::smooth(*this, true);
	befgen=new common::generator::bias_edp_force(*this);
	gagen=new common::generator::tff_gripper_approach (*this, 8);	//gripper approach constructor (task&, no_of_steps)
	sleepgen=new common::generator::sleep(*this);
	aitrans=new ecp_mp::transmitter::TRDraughtsAI(ecp_mp::transmitter::TRANSMITTER_DRAUGHTSAI,"[transmitter_draughts_ai]",*this);

	follower_vis = new ecp_vis_ib_eih_follower_irp6ot(*this);	//planar servomechanism generator
	follower_vis->sensor_m = sensor_m;
};

/*============================Destructor=====================================*/
Draughts::~Draughts(){
	delete ecp_m_robot;
	delete sgen;
	delete befgen;
	delete gagen;
	delete sleepgen;
	delete aitrans;
	delete follower_vis;
};

/*=============================bPawn2bKing===================================*/
void Draughts::bPawn2bKing(int from, int to){
	if(bkings>8){
		printf("there is no more kings available");
		return;
	}
	throwPawn(from);
	takeStaticPawn(bkings,BKING);
	bkings++;
	putStaticPawn(to);
}

/*=============================closeGripper==================================*/
void Draughts::closeGripper(){
	sgen->reset();
	sgen->set_relative();
	sgen->load_coordinates(lib::MOTOR, 0,0,0,0,0,0,0,-1000);
	sgen->Move();
}

/*===============================fradiaControl=================================*/
void Draughts::fradiaControl(lib::DRAUGHTS_MODE dmode, char pawn_nr=-1){
	lib::ECP_VSP_MSG msg;
	ecp_mp::sensor::cvfradia* vsp_fr=(ecp_mp::sensor::cvfradia*) sensor_m[lib::SENSOR_CVFRADIA];
	msg.draughts_control.draughts_mode=dmode;
	msg.draughts_control.pawn_nr=pawn_nr;
	vsp_fr->send_reading(msg);
	snooze(0.4);						//wait until signal is read
}

/*===========================gatAIMove=======================================*/
//1 get Black move - blue
//0 get White move - red
void Draughts::getAIMove(int player){
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	vsp_fradia->get_reading();
	while(vsp_fradia->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
			vsp_fradia->get_reading();
	}

	int port=config.return_int_value("ai_port","[AI]");
	std::string node_name=config.return_string_value("ai_node_name","[AI]");

	aitrans->AIconnect(node_name.c_str(),port);

	vsp_fradia->get_reading();
	printf("dane: \n");
	for(int i=0;i<32;i++){
		aitrans->to_va.draughts_ai.board[i]=vsp_fradia->from_vsp.comm_image.sensor_union.board.fields[i];
		printf("%d ",aitrans->to_va.draughts_ai.board[i]);

	}
	aitrans->to_va.draughts_ai.player=player;
	printf("\n");
	aitrans->t_write();
	aitrans->t_read(false);
	//printf("result %s ,%d\n",aitrans->from_va.draughts_ai.move,aitrans->from_va.draughts_ai.status);
	//strcpy(result,aitrans->from_va.draughts_ai.move);

	aitrans->AIdisconnect();
}

/*===============================getBoardStatus================================*/
lib::BOARD_STATUS Draughts::getBoardStatus(){
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	vsp_fradia->get_reading();
	while(vsp_fradia->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
			vsp_fradia->get_reading();
	}

	vsp_fradia->get_reading();
	return vsp_fradia->from_vsp.comm_image.sensor_union.board.status;
}

/*============================goToInitialPos=================================*/
void Draughts::goToInitialPos(){
	sgen->reset();
	sgen->set_absolute();
	sgen->load_coordinates(lib::MOTOR,0,-9,12,-49,151,-12,749,-1800);
	sgen->Move();
}

/*===============================goUp=======================================*/
void Draughts::goUp(){
	sgen->reset();
	sgen->set_relative();
	sgen->load_coordinates(lib::MOTOR,0,0,0,-20,0,0,0,0);	//move up
	sgen->Move();
}

/*============================init_tdes======================================*/
//inicjacja struktury tdes - trajectory description
void Draughts::init_tdes(lib::POSE_SPECIFICATION pspec, int internode_no){
	tdes.arm_type=pspec;
	tdes.interpolation_node_no=1;
	tdes.internode_step_no=internode_no;	//motion time
	tdes.value_in_step_no=internode_no-2;			//motion time-2 ??
}

/*=========================main_task_algorithm===============================*/
void Draughts::main_task_algorithm(void){
	wkings=0;
	bkings=0;
	goToInitialPos();
	fradiaControl(lib::DETECT_BOARD_STATE);
	uint8_t choice;
	choice=choose_option ("Do you want to play: 1 - Black(blue), or 2 - White(red)", 2);
	if (choice==lib::OPTION_ONE){
		sr_ecp_msg->message("You will play black, while I will play white");
		printf("You will play black, while I will play white\n");
		while(true){
			wait4move();
			if(makeAIMove(0)!=AI_NORMAL_MOVE)
				break;
		}
	}else if(choice==lib::OPTION_TWO){
		while(true){
			sr_ecp_msg->message("You will play white, while I will play black");
			printf("You will play white, while I will play black\n");
			if(makeAIMove(1)!=AI_NORMAL_MOVE)
				break;
			wait4move();
		}
	}else{
		sr_ecp_msg->message("Game canceled");
	}

	//takeDynamicPawn(31);

	fradiaControl(lib::NONE);
	ecp_termination_notice();
};

/*===============================makeAIMove========================================*/
int Draughts::makeAIMove(int player){
	char* move;
	char* board;
	int to;
	int movesnr;

	sleep(2);
	getAIMove(player);

	//printf("status: %d \nincoming: ",aitrans->from_va.draughts_ai.status);
	switch(aitrans->from_va.draughts_ai.status){
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

	move=aitrans->from_va.draughts_ai.move;
	board=aitrans->to_va.draughts_ai.board;

	//count number of jumper fields

	for(movesnr=0;movesnr<25;movesnr++){
		printf("%d ",move[movesnr]);
		if(move[movesnr]==-1){
			break;
		}
	}

	movesnr--;
//	printf("\nmovesnr: %d\n\n",movesnr);

	//chenge to dame
//	printf("change: %d %d\n",move[movesnr],board[move[0]]);
	if((move[movesnr]<4 || move[movesnr]>27) && board[move[0]]>1 ){
		if(board[move[0]]==2){
			wPawn2wKing(move[0],move[movesnr]);
			printf("change to whtie dame\n");
		}else if(board[move[0]]==3){
			printf("change to black dame\n");
			bPawn2bKing(move[0],move[movesnr]);
		}
	}else{
		//make move
		movePawn(move[0],move[movesnr]);
	}

	//remove jumped pawns
	if(movesnr>1){
		for(int i=0;i<movesnr;i++)
			if(i%2)
				throwPawn(move[i]);
	}
	goToInitialPos();

	if(aitrans->from_va.draughts_ai.status==AI_COMPUTER_WON)
		return AI_COMPUTER_WON;

	return AI_NORMAL_MOVE;
}

/*=======================movePawn============================================*/
void Draughts::movePawn(int from, int to){
	printf("moving pawn\n");
	//takeStaticPawn(from,PAWN);
	takeDynamicPawn(from);
	putStaticPawn(to);
}

/*===============================putStaticPawn===============================*/
void Draughts::putStaticPawn(int to){
	printf("putting pawn\n");

	sgen->reset();
	sgen->set_absolute();
	sgen->load_coordinates(lib::MOTOR,
			moves_table[to][0],moves_table[to][1],moves_table[to][2],moves_table[to][3],
			moves_table[to][4],moves_table[to][5],moves_table[to][6],GRIPPER_CLOSED);
	sgen->load_coordinates(lib::MOTOR,
				moves_table[to][0],moves_table[to][1],moves_table[to][2],moves_table[to][3],
				moves_table[to][4],moves_table[to][5],moves_table[to][6],GRIPPER_OPENED);
	sgen->Move();
	goUp();
}

/*=============================set_tdes======================================*/
void Draughts::set_tdes(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6){
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
void Draughts::snooze(double time){
	sleepgen->init_time(time);
	sleepgen->Move();
}

/*==============================takeDynamicPawn===============================*/
void Draughts::takeDynamicPawn(int from){
	printf("taking pawn - servo\n");
	goToInitialPos();
	trackPawn(from);
	befgen->Move();

	//configuration of gripper approach configure(speed, time_period)
	gagen->configure(0.01,300);
	gagen->Move();

	sgen->reset();
	sgen->set_relative();
	sgen->load_coordinates(lib::MOTOR,0,0,0,-0.5,0,0,0,0);	//go up to not touch board
	sgen->load_coordinates(lib::MOTOR,0,0,0,0,0,0,0,1800+GRIPPER_CLOSED);	//close gripper
	sgen->Move();
	goUp();

}

/*==============================takeStaticPawn===============================*/
void Draughts::takeStaticPawn(int from,int type){

	printf("taking pawn\n");
	sgen->reset();
	sgen->set_absolute();

	if(type==PAWN){
		sgen->load_coordinates(lib::MOTOR,
			moves_table[from][0],moves_table[from][1],moves_table[from][2],moves_table[from][3],
			moves_table[from][4],moves_table[from][5],moves_table[from][6],moves_table[from][7]);
	}else if(type==WKING){
		sgen->load_coordinates(lib::MOTOR,
			wkings_table[from][0],wkings_table[from][1],wkings_table[from][2],wkings_table[from][3],
			wkings_table[from][4],wkings_table[from][5],wkings_table[from][6],wkings_table[from][7]);
	}else if(type==BKING){
		sgen->load_coordinates(lib::MOTOR,
			bkings_table[from][0],bkings_table[from][1],bkings_table[from][2],bkings_table[from][3],
			bkings_table[from][4],bkings_table[from][5],bkings_table[from][6],bkings_table[from][7]);
	}

	sgen->Move();

	befgen->Move();

	//configuration of gripper approach configure(speed, time_period)
	gagen->configure(0.01,300);
	gagen->Move();

	sgen->reset();
	sgen->set_relative();
	sgen->load_coordinates(lib::MOTOR,0,0,0,-0.5,0,0,0,0);	//go up to not touch board
	sgen->load_coordinates(lib::MOTOR,0,0,0,0,0,0,0,1800+GRIPPER_CLOSED);	//close gripper
	sgen->Move();
	goUp();
}

/*===============================throwPawn===================================*/
void Draughts::throwPawn(int from){
	takeDynamicPawn(from);
	//takeStaticPawn(from,PAWN);
	sgen->reset();
	sgen->set_absolute();
	sgen->load_coordinates(lib::MOTOR,0,45,30,21,151,-11,649,GRIPPER_CLOSED);		//go beyond board
	sgen->load_coordinates(lib::MOTOR,0,45,30,21,151,-11,649,GRIPPER_OPENED);		//throw pawn
	sgen->Move();
}

/*=======================trackPawn===========================================*/
void Draughts::trackPawn(char pawn_nr){
	fradiaControl(lib::NONE);
	fradiaControl(lib::TRACK_PAWN,pawn_nr);
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	vsp_fradia->get_reading();
	while(vsp_fradia->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
			vsp_fradia->get_reading();
	}

	sr_ecp_msg->message("Przed follower_vis");
	follower_vis->Move();
	sr_ecp_msg->message("Po follower_vis");
}

/*============================wait4move=======================================*/
void Draughts::wait4move(){
	lib::BOARD_STATUS status;
	fradiaControl(lib::STORE_BOARD);	//store board

	do{
		fradiaControl(lib::NONE);			//do not analyze board for a while
		snooze(1);							//wait 2 seconds and..
		fradiaControl(lib::CHECK_MOVE);
		status=getBoardStatus();			//check if state of the board chagned
		switch(status){
			case lib::STATE_CHANGED:
				printf("state changed\n");
				break;
			case lib::STATE_UNCHANGED:
				printf("state unchanged\n");
				break;
			case lib::BOARD_DETECTION_ERROR:
				printf("board detection error\n");
				break;
		}
	}while(status!=lib::STATE_CHANGED);

	fradiaControl(lib::DETECT_BOARD_STATE);
}

/*=============================wPawn2wKing===================================*/
void Draughts::wPawn2wKing(int from, int to){
	if(wkings>8){
		printf("there is no more kings available");
		return;
	}

	throwPawn(from);
	takeStaticPawn(wkings,WKING);
	wkings++;
	putStaticPawn(to);
}
}  //namespace task
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6ot::task::Draughts(_config);

}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

