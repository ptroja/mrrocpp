#define CMVISION
#include "./global.h"
#include "./calib.h"
#include "./macierze_nr.h"
#include <stdio.h>
#include <stdlib.h>
#include<math.h>
#include "lib/mathtr.h"

#define CMV_COLOR_LEVELS  256
#define CMV_MAX_COLORS     32

// sets tweaked optimal values for image size
#define CMV_DEFAULT_WIDTH  720
#define CMV_DEFAULT_HEIGHT 576

// values may need tweaked, although these seem to work usually
#define CMV_MAX_RUNS     (CMV_DEFAULT_WIDTH * CMV_DEFAULT_HEIGHT) / 2
#define CMV_MAX_REGIONS  CMV_MAX_RUNS / 2

#define CMV_NONE ((unsigned)(-1))

#define BLOB_SIZE_BIGGER 1
#define BLOB_SIZE_SMALLER 2
#define BLOB_CIRCULARITY_BIGGER 3
#define BLOB_CIRCULARITY_SMALLER 4
#define CONTOUR_LENGTH_BIGGER 5
#define CONTOUR_LENGTH_SMALLER 6
#define VERTICES_BIGGER 7
#define VERTICES_SMALLER 8
#define NEIGHBORS 9

#define AUTO 999 //oznacza ze dany algorytm ma automatycznie dobrac jakis parametr

#define SQUARE_RATIO 0.5625

#define WINDOW_COLOR 5

#define PATTERNSMAX 1728 //12^3;     2197=13^3
#define TILESMAX 27

#define SHIFT_X 5
#define SHIFT_Y 5
#define SHIFT_Z 5

#define POINT //stala aby nie definiowac ponizszych struktur w kolejnych plikach
struct point {
		int x;
		int y;
};

struct point_d {
		double x;
		double y;
};

struct filter{
	int area_min;
	int area_max;
	double circularity_min;
	double circularity_max;
	int contour_min;
	int contour_max;
	int vertices_min;
	int vertices_max;
	int neighbors;
};

#define ROI //stala aby nie definiowac ponizszej struktury w kolejnych plikach
struct roi_rubik{
		int x1; //wspolrzedne obszaru kostki
		int y1;
		int x2;
		int y2;
		point_d center; //srodek
		int area;		//powierzchnia
		int ratio;		//stosunek bokow
	};


struct hsv{
  int h,s,v;	/*moje*/
};

typedef struct hsv image_pixel; /*moje*/

struct rgb{
  int r,g,b;
};

struct rectangle{
		int x1;
		int y1;
		int x2;
		int y2;
	};

class CMVision{
public:




  struct rle{
    unsigned color;     // which color(s) this run represents
    int length;         // the length of the run (in pixels)
    int parent;         // run's parent in the connected components tree
  };

  struct color_info{
    rgb color;          // example color (such as used in test output)
    char *name;         // color's meaninful name (e.g. ball, goal)
    int class_num;     // numer klasy kolorow
    int h_low,h_high;   // H,S,V component thresholds
    int s_low,s_high;
    int v_low,v_high;
  };


  struct QpBlob{
    int color;          // id of the color
    int area;           // occupied area in pixels
    int x1,y1,x2,y2;    // bounding box (x1,y1) - (x2,y2)
    point_d center;  // centroid

    int sum_x,sum_y,sum_z; // temporaries for centroid and avg color

    QpBlob *next;       // next region in list
	point start;		// witek - punkt startowy bloba
	unsigned int contour_length;
	unsigned int vertices_count; //liczba wierzcholkow - liczona dodatkowo!
	point vertices[10]; //wektor przechowujacy wsp. wiercholkow - miesci niewiele, liczony dodatkowo!

  };

struct QpVertices {
		unsigned int points[1000];
		unsigned int count;
};


  struct QpContour {
	point points[80000];
	unsigned int length;
};

	QpBlob blobs[CMV_MAX_REGIONS];
	unsigned int filtered_blobs[CMV_MAX_REGIONS];
	unsigned char valid_blobs[CMV_MAX_REGIONS];
	unsigned char goal_blobs[CMV_MAX_REGIONS];
	unsigned char endeffector_blobs[CMV_MAX_REGIONS];

	unsigned int filtered_blobs_count;
	unsigned int blobs_count;

	unsigned char goal_blobs_count;
	unsigned char endeffector_blobs_count;


	QpContour contour;
	QpVertices vertices;


    //unsigned *map;

    int width,height;
	rectangle roi;

protected:
  unsigned h_class[CMV_COLOR_LEVELS];
  unsigned s_class[CMV_COLOR_LEVELS];
  unsigned v_class[CMV_COLOR_LEVELS];

  // returns index of least significant set bit
int log2modp[37];

// witek - do laczenia blobow o roznych kolorach
unsigned int color_tab[37];

//MACIEK
unsigned char RGB2H[0xffff];
unsigned char RGB2S[0xffff];
unsigned char RGB2V[0xffff];

public:
  unsigned *map; //should be private
  unsigned *labels;
  rle rmap[CMV_MAX_RUNS];

  color_info colors[CMV_MAX_COLORS];

  int face_colors[9];
  int face_h[9];
  int face_s[9];
  int face_v[9];

  int static_middle_grid_xa[9];//={173,207,242,138,173,209,105,137,173};
  int static_middle_grid_ya[9];//={72,107,142,107,142,178,142,178,214};

  FILE *pattern_fp;
  point_d mappattern[PATTERNSMAX][TILESMAX];

  int whichpattern[PATTERNSMAX];
  int tiles_count[PATTERNSMAX];


  point_d goal_f1;
  point_d goal_f2;
  point_d goal_f3;
  point_d goal_f4;

  point_d endeffector_f1;
  point_d endeffector_f2;
  point_d endeffector_f3;
  point_d endeffector_f4;

  point_d error_f1;
  point_d error_f2;
  point_d error_f3;
  point_d error_f4;


  //effector
	double endeffector_meanx;
	double endeffector_meany;



  int endeffector_matched_pattern;
  double endeffector_matched_Z;



  int whole_face;

  double goal_meanx;
  double goal_meany;

  int goal_matched_pattern;
  double goal_matched_Z;

  double *cc, *fc, *kc; //globalnie widoczne parametry kamery
  int alloc_m;
  int alloc_v; //globalnie widoczne liczby zaalokowanych macierzy i wektorow
  double **x_kk, **X_kk;
  double *omckk;
  double *Tckk;
  double **Rckk;

  double C_T_E[4][4]; // ^{C}T_{E}
  double C_T_G[4][4]; // ^{C}T_{G} // ^{C}T_{E}
  double E_T_G__C[4][4]; // ^{E}T_{G}= \, (^{C}T_{G}) \,^{-1} \, ^{C}T_{G}

  double C_r_E[6];  // ^{C}r_{E}
  double C_r_G[6];  // ^{C}r_{G}
  double C_eps_EG[6];  // ^{C}\vareps_{E,G}

  lib::Homog_matrix C_Tx_G;
lib::Homog_matrix C_Tx_E;
lib::Homog_matrix O_Tx_E;
lib::Homog_matrix O_Tx_C;
lib::Homog_matrix O_Tx_E__C;
lib::Homog_matrix O_Tx_G__C;

lib::Homog_matrix E_Tx_G;
lib::Homog_matrix E_Tx_G__O;

  //double *cube_vector;
  //double *cube_center;
  //double *cube_temp;
//protected:
public:
// Private functions

  void connectComponents(rle * rmap,int num);
  int  extractRegions(rle * rmap,int num);
  //void classifyFrame(image_pixel * img,unsigned * map);

  void classifyFrame(unsigned short * buffer, unsigned * map);
  int  encodeRuns(rle * out,unsigned * map);

  void classifyFace(unsigned short * buffer);

  void clear();

public:
  CMVision()  {clear();}
  ~CMVision() {close();}

//unsigned int labels[1000][1000]; //witek do tworzenia obrazu etykiet
									// da sie to obejsc i nie spowalniac

  bool initialize(int nwidth,int nheight);

  bool initGrid();

  bool initEstim(const char *filename); //init estimation
  void freeEstim();

  bool countLUT();

  bool loadColors(const char *filename);

  void close();


 // bool findBlobs(image_pixel *image);
 bool findBlobs(unsigned short * buffer);
 // bool findBlobsfilter(image_pixel *image, filter data);

int findContour(int blob_nr);
void findVertices(int dokl);
void findVerticesAll(void);

void filterBlobsReset(void);
bool filterBlobs(int filter, double value);

void setRoi(roi_rubik r, int m);

//void findCorners();
bool estimPose();
//bool estimPose2();
bool objectClass();

bool estimPose2(int is_effector, double meanx, double meany, double &matched_Z, int &matched_pattern, point_d &ff1, point_d &ff2, point_d &ff3, point_d &ff4);

bool estimPose3();
bool estimPose4();

bool estimError();

protected:
unsigned char contour_point(int x, int y, unsigned int nr);
void znajdz_najdalszy(int p1, int p2, int kwadrat_odl, unsigned char wszystkie);
int findVertices(int dokl, int blob_nr);



 };
