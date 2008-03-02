#include <fstream>
#include <string>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "lib/nurbs_tdes.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_nurbs.h"
#include "ecp/irp6_on_track/ecp_t_ts_irp6ot.h"

std::ifstream& operator>>(std::ifstream& s, std::valarray<double>& v) {
	size_t i=0;
	while (i<v.size() && s>>v[i]) i++;
	return s;
} 

 

const size_t Dim=6; //jest juz ecp_gen

#include "lib/nurbs.h"
using namespace NurbsLib;

template< POSE_SPECIFICATION arm_type, size_t D >
class Irp6ot_Point_nD : public Point_nD< D > {; };

NurbsCurve< Irp6ot_Point_nD< XYZ_EULER_ZYZ, Dim> > nurbsc;

extern ostream& operator<<(ostream& s, const valarray<double>& v);


// KONSTRUKTORY
ecp_task_ts_irp6ot::ecp_task_ts_irp6ot(configurator &_config) : ecp_task(_config)
{

};

ecp_task_ts_irp6ot::~ecp_task_ts_irp6ot(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_ts_irp6ot::task_initialization(void) 
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_ts_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP - nurbs - press start button");
	ecp_wait_for_start();
   for(;;) { // Wewnetrzna petla nieskonczona
   // Aktualnie petla wykonuje sie jednokrotnie, gdyby MP przejal sterowanie
   // to petle mozna przerwac przez STOP lub przez polecenie END_MOTION wydane
   // przez MP

	nurbs_tdes ntdes;
//	ntdes.arm_type = XYZ_EULER_ZYZ;
 //	tdes.interpolation_node_no = 500;
	ntdes.internode_step_no = 10;
	ntdes.value_in_step_no = ntdes.internode_step_no - 1;
	
/*	int chid;
	chid = ChannelCreate(0);
	pid_t pid = getpid();
	
	char charpid[8 * sizeof( int ) + 1];
	char charchid[8 * sizeof( int ) + 1];
	spawnl (P_NOWAIT, "./nurbs", "./nurbs", itoa(pid,charpid,10), itoa(chid,charchid,10), NULL);
	cout<<"\nclient: "<<charpid<<"   "<<charchid<<flush;
*/	
	const char *filenurbs="trajektoria.nrb"  ;
	
	ofstream ofile(filenurbs);
	if (!ofile) cerr<<"Nie mozna otworzyc pliku wyjsciowego: "<<*filenurbs<<"\n"<<flush;
	ofile<<"NURBS: degree, number of control points, knot vector, control points with weights\n";
	ofile<<"2\n";
	ofile<<"10\n";
	ofile<<"0	0	0	1.25	2.5	3.75	5	6.25	7.5	8.75	10	10	10	\n";
	ofile<<"0	0	0	0	0	0	1	\n";
	ofile<<"0	0	0	0	0	0	1	\n";
	ofile<<"0	0.05	0	0	0	0	1	\n";
	ofile<<"0	0.05	0	0	0	0	1	\n";
	ofile<<"0.05	0.05	0	0	0	0	1	\n";
	ofile<<"0.05	0.05	0	0	0	0	1	\n";
	ofile<<"0.05	0	0	0	0	0	1	\n";
	ofile<<"0.05	0	0	0	0	0	1	\n";
	ofile<<"0	0	0	0	0	0	1	\n";
	ofile<<"0	0	0	0	0	0	1	\n";
	
	ofile.close();
	
	
	using namespace NurbsLibImp;

	while(1) {	
		ifstream ifile(filenurbs);
			if (!ifile) cerr<<"Nie mozna otworzyc pliku zrodlowego: "<<*filenurbs<<"\n"<<flush;
	
		//wczytaj i ustaw stopien krzywej
		string s;
		ifile>>s;
		char ch;
		while (ifile.get(ch))
			if (ch=='\n') break;

		size_t degree;
		if (s==string("Interpolation:")) {
			cout<<"Interpolation not implemented in this ecp\n";/*
			ifile>>degree;

			const size_t size=2;
			vector< valarray<double> > Qv(size, valarray<double>(0.0, Dim+1));
			vector<double> ub(size, 0.0 );
			while (!ifile.eof()) {
				valarray<double> v(Dim);		double u;
				ifile>>v>>u;		
				Qv.push_back(v);	 ub.push_back(u);}
			nurbsc.globalInterp(Qv, ub, degree);*/
		}
		else { //Control points
			ifile>>degree;
			size_t n;
			ifile>>n;
			nurbsc.resize(n,degree);
			for (size_t i=0; i<n+degree+1; i++) {
				double d;
				ifile>>d;
				nurbsc.knotVector()[i]=d; }
			for (size_t i=0; i<n; i++) {
				valarray<double> d(Dim+1);
				ifile>>d;
				nurbsc.pointsVector()[i].vala()=d;}
	//		cout<<endl<<"T:"<<vT<<endl<<flush;
		}
	
		ifile.close();
	
		//zapis krzywej do pliku
		const char *filename="./nurbs_trj.txt"  ;
	
		ofstream ofile(filename);
		if (!ofile) cerr<<"Nie mozna otworzyc pliku wyjsciowego: "<<*filename<<"\n"<<flush;

		double d=nurbsc.minT();
		const double dmax=nurbsc.maxT();
		while (d<dmax) {
			ofile<<nurbsc.curvePoint(d).cvala()[Point_nD<6>::mask]<<"\n";
			d+=0.002;}
	
		ofile.close();
		
	
			
		ntdes.ncptr= &nurbsc;	
	 	ntdes.interpolation_node_no = (int) (nurbsc.maxT()*50);
		irp6ot_nurbs_generator ng(*this, ntdes,0);
		ng.sensor_m = sensor_m;
	  	sr_ecp_msg->message("Generator created.");
		Move ( ng);
	} //end while(1)
	 
	 
//	 printf("w ecp for za move\n");
     // Oczekiwanie na STOP
     ecp_wait_for_stop();
     break; // W.S. ??? czy powinna byc ta instrukcja
   } // koniec: for(;;) wewnetrznej
};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_ts_irp6ot(_config);
};
