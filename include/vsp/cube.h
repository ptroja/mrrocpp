#ifndef CMVISION
#include "cmvision.h"
#endif

#define RUBIK_MAX_TILES 20


#ifndef POINT
#define POINT
struct point {
		int x;
		int y;
};

struct point_d {
		double x;
		double y;
};
#endif

#ifndef ROI
struct roi_rubik{
		int x1; //wspolrzedne obszaru kostki
		int y1;
		int x2;
		int y2;
		point_d center; //srodek
		int area;		//powierzchnia
		int ratio;		//stosunek bokow
	};
#endif

struct rubik_compare{
	int tiles_count_diff;
	int roi_area_ratio;
	int tiles_old_count;
	int tiles_new_count;
	int tiles[RUBIK_MAX_TILES];
	

};

struct tilesT{
	
	int x1;
	int y1;
	int x2;
	int y2; //wsp. prostokata ograniczajacego
	int area;
	int contour_length;
	int color;
	point vertices[4];
	point_d center; //srodek ciezkosci calego bloba
	point_d center_v; // srodek ciezkosci czworokata
};

class RubiksCube{
public:


 struct stats{
	int min;
	int max;
	int average;
  };


  roi_rubik roi;
  int tiles_count;
  tilesT tiles[RUBIK_MAX_TILES];
  stats tiles_area;
  stats tiles_contour;
  
 
  

	
	

public:
  RubiksCube()  {clear();}
  ~RubiksCube() {close();}

  

  void close();
  void clear();

rubik_compare RubiksCube::compare(RubiksCube *k);
int RubiksCube::findTile(int nr,RubiksCube *k);
point_d RubiksCube::findShift(RubiksCube *k);
bool RubiksCube::build(CMVision *vision);

  

 };

