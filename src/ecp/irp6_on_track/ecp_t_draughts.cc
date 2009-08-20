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
	{0.000000,   6.762255, -40.591957, 49.588738, 151.072834, -5.583006, 719.987063, -1800},
	{0.000000,  -9.507694, -43.171564, 49.100455, 150.879363, -5.813328, 749.800777, -1800},
	{0.000000, -25.878985, -42.830687, 49.045178, 150.676680, -6.062076, 779.796704, -1800},
	{0.000000, -41.734354, -38.905999, 49.515035, 150.501635, -6.320037, 808.828161, -1800},
	{0.000000,  13.303401, -22.838734, 51.385250, 151.174176, -5.693561, 708.121267, -1800},
	{0.000000,  -1.575403, -26.542312, 51.035160, 150.989918, -5.850180, 735.440557, -1800},
	{0.000000, -17.182024, -27.841328, 50.823264, 150.814873, -6.034438, 763.991351, -1800},
	{0.000000, -32.696517, -25.713153, 50.878541, 150.658254, -6.237121, 792.388207, -1800},
	{0.000000,   4.836763, -10.576388, 51.901171, 151.082047, -5.776477, 723.694142, -1800},
	{0.000000,  -9.968338, -12.722990, 51.772191, 150.943853, -5.951522, 750.746396, -1800},
	{0.000000, -24.865568, -12.446603, 51.707701, 150.796447, -6.126566, 778.015421, -1800},
	{0.000000, -39.265302,  -9.019411, 51.744552, 150.621403, -6.301611, 804.329401, -1800},
	{0.000000,  10.990968,   5.887031, 52.011726, 151.128111, -5.840967, 712.177063, -1800},
	{0.000000,  -2.773077,   2.791503, 52.057790, 151.008344, -5.933096, 737.419760, -1800},
	{0.000000, -17.006980,   1.556977, 51.744552, 150.860938, -6.062076, 763.535820, -1800},
	{0.000000, -31.019773,   3.325850, 51.606359, 150.722744, -6.209482, 789.158650, -1800},
	{0.000000,   3.288999,  17.836139, 51.182567, 151.063621, -5.923883, 726.329938, -1800},
	{0.000000, -10.253937,  15.643473, 51.274695, 150.925428, -6.071289, 751.054273, -1800},
	{0.000000, -23.815299,  16.187033, 51.200992, 150.778022, -6.218695, 775.907412, -1800},
	{0.000000, -36.934443,  18.610021, 50.961457, 150.649041, -6.375314, 799.830640, -1800},
	{0.000000,   8.825941,  32.807072, 49.183371, 151.100472, -5.868606, 716.151178, -1800},
	{0.000000,  -3.906262,  30.089272, 49.616377, 150.953066, -5.997586, 739.380114, -1800},
	{0.000000, -16.730593,  29.131132, 49.699293, 150.824086, -6.135779, 762.891794, -1800},
	{0.000000, -29.785246,  30.734173, 49.386055, 150.704319, -6.283185, 786.683075, -1800},
	{0.000000,   1.814938,  43.899381, 47.064408, 151.035982, -5.942309, 728.994009, -1800},
	{0.000000, -10.751433,  42.001528, 47.396072, 150.897789, -6.071289, 751.943343, -1800},
	{0.000000, -23.262526,  42.517449, 47.174963, 150.778022, -6.200269, 774.845554, -1800},
	{0.000000, -35.322188,  44.958863, 46.631403, 150.676680, -6.301611, 796.952941, -1800},
	{0.000000,   6.706978,  57.635788, 42.434533, 151.155750, -5.850180, 720.323213, -1800},
	{0.000000,  -4.919679,  55.332567, 44.070222, 151.035982, -5.951522, 741.607503, -1800},
	{0.000000, -16.666103,  54.835072, 44.208415, 150.897789, -6.098928, 762.872944, -1800},
	{0.000000, -28.900810,  55.986682, 42.895177, 150.741170, -6.246334, 785.193960, -1800},
	};

const double Draughts::wkings_table[4][8]={
	{0.000000, 34.354836, -33.516464, 48.736144, 151.386072, -5.610645, 669.746713, -1800.034416},
	{0.000000, 30.789451, -10.336853, 50.237844, 151.395285, -5.619858, 676.161845, -1800.034416},
	{0.000000, 27.804477,  11.304206, 49.749561, 151.367646, -5.656709, 681.691048, -1799.985328},
	{0.000000, 25.188019,  32.088467, 47.400276, 151.349220, -5.665922, 686.541667, -1800.034416}
};

const double Draughts::bkings_table[4][8]={
	{0.000000, -49.574516, 68.728097, 37.901795, 150.612190, -6.402953, 823.207231, -1800.034416},
	{0.000000, -51.721118, 49.215214, 43.576930, 150.602977, -6.402953, 827.171921, -1800.034416},
	{0.000000, -54.227022, 29.103493, 47.529257, 150.575338, -6.439804, 831.654974, -1800.03441},
	{0.000000, -56.944822,  8.374509, 49.546878, 150.547700, -6.458230, 836.722363, -1800.034416}
};



/*==============================Constructor==================================*/
//Constructors
Draughts::Draughts(lib::configurator &_config): task(_config){
	sgen=NULL;
	befgen=NULL;
	gagen=NULL;
	lgen=NULL;
	aitrans=NULL;
	bkings=0;
	wkings=0;
};

/*============================Destructor=====================================*/
Draughts::~Draughts(){
	delete ecp_m_robot;
	delete sgen;
	delete befgen;
	delete gagen;
	delete go_st;
	delete sleepgen;
	delete aitrans;
	delete planar_vis;
};

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

/*=======================task_initialization=================================*/
// methods for ECP template to redefine in concrete classes
void Draughts::task_initialization(void){

	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA,"[vsp_cvfradia]", *this,	sizeof(lib::sensor_image_t::sensor_union_t::fradia_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	ecp_m_robot=new robot(*this);				//initialization of robot
	sgen=new common::generator::smooth(*this, true);
	befgen=new common::generator::bias_edp_force(*this);
	gagen=new common::generator::tff_gripper_approach (*this, 8);	//gripper approach constructor (task&, no_of_steps)
	go_st = new common::task::ecp_sub_task_gripper_opening(*this);
	sleepgen=new common::generator::sleep(*this);
	aitrans=new ecp_mp::transmitter::TRDraughtsAI(ecp_mp::transmitter::TRANSMITTER_DRAUGHTSAI,"[transmitter_draughts_ai]",*this);

	planar_vis = new ecp_vis_ib_eih_planar_irp6ot(*this);	//planar servomechanism generator
	planar_vis->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP loaded tb");
};



/*=======================trackPawn===========================================*/
void Draughts::trackPawn(){
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	vsp_fradia->get_reading();
	while(vsp_fradia->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
			vsp_fradia->get_reading();
	}

	sr_ecp_msg->message("Przed planar_vis");
	planar_vis->Move();
	sr_ecp_msg->message("Po planar_vis");
}

/*=======================movePawn============================================*/
void Draughts::movePawn(int from, int to){
	printf("moving pawn\n");
	takeStaticPawn(from,PAWN);
	putStaticPawn(to);
}

/*=============================closeGripper==================================*/
void Draughts::closeGripper(){
	sgen->reset();
	sgen->set_relative();
	sgen->load_coordinates(lib::MOTOR, 0,0,0,0,0,0,0,-1000);
	sgen->Move();
}

/*==============================takeStaticPawn===============================*/
/* const int WKING =0, BKING=1;
/* const int PAWN =6
 */
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

/*===============================putStaticPawn===============================*/
void Draughts::goUp(){
	sgen->reset();
	sgen->set_relative();
	sgen->load_coordinates(lib::MOTOR,0,0,0,-20,0,0,0,0);	//move up
	sgen->Move();
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

/*============================goToInitialPos=================================*/
void Draughts::goToInitialPos(){
	sgen->reset();
	sgen->set_absolute();
	sgen->load_coordinates(lib::MOTOR,0,-9,12,-49,151,-6,749,-1800);
	sgen->Move();
}

/*===============================throwPawn===================================*/
void Draughts::throwPawn(int from){
	takeStaticPawn(from,PAWN);
	sgen->reset();
	sgen->set_absolute();
	sgen->load_coordinates(lib::MOTOR,0,45,30,21,151,-5,649,GRIPPER_CLOSED);		//go beyond board
	sgen->load_coordinates(lib::MOTOR,0,45,30,21,151,-5,649,GRIPPER_OPENED);		//throw pawn
	sgen->Move();
}

/*=============================wPawn2wKing===================================*/
void Draughts::wPawn2wKing(int from, int to){
	if(wkings>4){
		printf("there is no more kings available");
		return;
	}

	throwPawn(from);
	takeStaticPawn(wkings,WKING);
	wkings++;
	putStaticPawn(to);
}


/*=============================bPawn2bKing===================================*/
void Draughts::bPawn2bKing(int from, int to){
	if(bkings>4){
		printf("there is no more kings available");
		return;
	}
	throwPawn(from);
	takeStaticPawn(bkings,BKING);
	bkings++;
	putStaticPawn(to);
}

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

/*===============================fradiaControl=================================*/
void Draughts::fradiaControl(lib::DRAUGHTS_MODE dmode){
	lib::ECP_VSP_MSG msg;
	ecp_mp::sensor::cvfradia* vsp_fr=(ecp_mp::sensor::cvfradia*) sensor_m[lib::SENSOR_CVFRADIA];
	msg.draughts_mode=dmode;
	vsp_fr->send_reading(msg);
	snooze(0.4);						//wait until signal is read
}

/*===============================snooze========================================*/
void Draughts::snooze(double time){
	sleepgen->init_time(time);
	sleepgen->Move();
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

/*=========================main_task_algorithm===============================*/
void Draughts::main_task_algorithm(void){
	wkings=0;
	bkings=0;
	goToInitialPos();
	fradiaControl(lib::DETECT_BOARD_STATE);
	lib::BYTE choice;
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

	//trackPawn();


	 /*double vp[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	 double vk[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	 double v[MAX_SERVOS_NR]={0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 5.0};
	 double a[MAX_SERVOS_NR]={0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.5};
	 double coordinates[MAX_SERVOS_NR];


	 coordinates[0]=0;
	 coordinates[1]=0;
	 coordinates[2]=0;
	 coordinates[3]=0;
	 coordinates[4]=0;
	 coordinates[5]=0;
	 coordinates[6]=0;
	 coordinates[7]=0;
	*/
	 //sgen->load_file_with_path("/net/grafika/home/tbem/trj/draughts/test2.trj");
	 //sgen->Move();

	//trackPawn(); track pawn in xy plane
	ecp_termination_notice();
};

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

/*============================init_tdes======================================*/
//inicjacja struktury tdes - trajectory description
void Draughts::init_tdes(lib::POSE_SPECIFICATION pspec, int internode_no){
	tdes.arm_type=pspec;
	tdes.interpolation_node_no=1;
	tdes.internode_step_no=internode_no;	//motion time
	tdes.value_in_step_no=internode_no-2;			//motion time-2 ??
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

