#include "vsp/vis/cmvision.h"
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <stdio.h>
#include <string.h>

#define HALF_CUBE 27 //25
#define BETWEEN_CENTERS 39.5 //39.5


inline int range_sum(int x,int w)
{
  return(w*(2*x + w-1) / 2);
}

// returns maximal value of two parameters
inline unsigned int max(unsigned int a,unsigned int b)
{
  return((a > b)? a : b);
}

// returns minimal value of two parameters

inline unsigned int min(unsigned int a,unsigned int b)
{
  return((a < b)? a : b);
}

inline unsigned int max3(unsigned int a,unsigned int b, unsigned int c)
{
	unsigned tmp=(a > b)? a : b;
  return((c > tmp)? c : tmp);
}

// returns minimal value of two parameters

inline unsigned int min3(unsigned int a,unsigned int b,unsigned int c)
{
	unsigned tmp=(a < b)? a : b;
  return((c < tmp)? c : tmp);
}


// returns index of least significant set bit
int log2modp[37] = {
  0, 1, 2,27, 3,24,28, 0, 4,17,25,31,29,12, 0,14, 5, 8,18,
  0,26,23,32,16,30,11,13, 7, 0,22,15,10, 6,21, 9,20,19
};

// witek - do laczenia blobow o roznych kolorach
int color_tab[37] = {
  0, 1, 2,27, 3,24,28, 0, 4,17,25,31,29,12, 0,14, 5, 8,18,
  0,26,23,32,16,30,11,13, 7, 0,22,15,10, 6,21, 9,20,19
};


// returns index of most significant set bit
inline int top_bit(int n)
{
  int i = 1;
  if(!n) return(0);
  while(n>>i) i++;
  return(i);
}



//void CMVision::classifyFrame(image_pixel * img,unsigned * map)
void CMVision::classifyFrame(unsigned short * buffer, unsigned * map)
// Classifies an image passed in as img, saving bits in the entries
// of map representing which thresholds that pixel satisfies.
{
  //int i,size;
  int y1,y2,x1,x2,m;
  // image_pixel p;
  int x,y;

  //unsigned *sclas = s_class; // Ahh, the joys of a compiler that
 // unsigned *vclas = v_class; //   has to consider pointer aliasing
 // unsigned *hclas = h_class;

 /*
  size = width * height;
  for(i=0; i<size; i++){
   p = img[i];
    //m = hclas[p.h] & sclas[p.s] & vclas[p.v];
   m = h_class[p.h] & s_class[p.s] & v_class[p.v];
	//map[i] = m;
	map[i]=color_tab[m%37];
  }
  */

  y1=roi.y1*width;
  y2=roi.y2*width;

  for(y=y1; y<=y2; y+=width)
  {
	x1=y+roi.x1;
	x2=y+roi.x2;
	for(x=x1; x<=x2; x++)
  {
//	p = img[x];

	m = h_class[RGB2H[buffer[x]]] & s_class[RGB2S[buffer[x]]] & v_class[RGB2V[buffer[x]]];
	m=color_tab[m%37];
	map[x]=m;
	}
  }

}

void CMVision::classifyFace(unsigned short * buffer)
// Classifies an image passed in as img, saving bits in the entries
// of map representing which thresholds that pixel satisfies.
{
  //int i,size;
  int y1,y2,x1,x2,m;
  // image_pixel p;
  int x,y;

  int H,S,V;
  H=0;
  S=0;
  V=0;

  for(int i=0; i<9; i++)
  {
	  y1=(static_middle_grid_ya[i]-WINDOW_COLOR)*width;
	  y2=(static_middle_grid_ya[i]+WINDOW_COLOR)*width;
	  for(y=y1; y<=y2; y+=width)
	  {
			x1=y+static_middle_grid_xa[i]-WINDOW_COLOR;
			x2=y+static_middle_grid_xa[i]+WINDOW_COLOR;
			for(x=x1; x<=x2; x++)
	  		{

				H+=RGB2H[buffer[x]];
				S+=RGB2S[buffer[x]];
				V+=RGB2V[buffer[x]];

		//m = h_class[RGB2H[buffer[x]]] & s_class[RGB2S[buffer[x]]] & v_class[RGB2V[buffer[x]]];
		//m=color_tab[m%37];
		//map[x]=m;
		//printf("HSV: %d %d %d\n",RGB2H[buffer[x]],RGB2S[buffer[x]],RGB2V[buffer[x]]);
 			}
	  }

	  H/=121;
	  S/=121;
	  V/=121;

	  printf("HSV: %d %d %d\n",H, S, V);

	  m = h_class[H] & s_class[S] & v_class[V];
	  m=color_tab[m%37];

	  face_colors[i]=m;

	  face_h[i]=H;
	  face_s[i]=S;
	  face_v[i]=V;



	 //printf("%d\n", m);
  }
  //printf("\n");
}

int CMVision::encodeRuns(rle * out,unsigned * map)
// Changes the flat array version of the threshold satisfaction map
// into a run length encoded version, which speeds up later processing
// since we only have to look at the points where values change.
{
  int x,y,j,l;
  unsigned m,save;
 // int size;
  unsigned *row;
  rle r;
  int width2;

  if(roi.x2<width-1)
	width2=roi.x2+1;
  else
	  width2=width;


  // initialize terminator restore
  save = map[roi.y1*width]; //bylo map[0]

  j = 0;
  //j=roi.y1*width+roi.x1;
 // for(y=0; y<height; y++){
  for(y=roi.y1; y<=roi.y2; y++){
    row = &map[y * width];

    // restore previous terminator and store next
    // one in the first pixel on the next row
    //row[0] = save;
	row[0] = save;
    //save = row[width];
	save = row[width2];
    //row[width] = CMV_NONE;
	row[width2] = CMV_NONE;

    //x = 0;
	x=roi.x1;
    //while(x < width){
	while(x < width2){
      m = row[x];
      l = x;
      while(row[x] == m) x++;

      r.color  = m;
	  r.length = x - l;
      r.parent = j;
      out[j++] = r;
      if(j >= CMV_MAX_RUNS)
		  return(0);
    }
  }

  return(j);
}

void CMVision::connectComponents(rle * rmap,int num)
// Connect components using four-connecteness so that the runs each
// identify the global parent of the connected region they are a part
// of.  It does this by scanning adjacent rows and merging where similar
// colors overlap.  Used to be union by rank w/ path compression, but now
// is just uses path compression as the global parent index seems to be
// a simpler fast approximation of rank in practice.
// WARNING: This code is *extremely* complicated and twitchy.  It appears
//   to be a correct implementation, but minor changes can easily cause
//   big problems.  Read the papers on this library and have a good
//   understanding of tree-based union find before you touch it
{
  int x1,x2;
  int l1,l2;
  rle r1,r2;
  int i,p,s,n;

  l1 = l2 = 0;
  //x1 = x2 = 0;
  x1=roi.x1;
  x2=0; //??

  // Lower scan begins on second line, so skip over first
  //while(x1 < width){
  while(x1 <= roi.x2){
    x1 += rmap[l1++].length;
  }
  x1 = 0;

  // Do rest in lock step
  r1 = rmap[l1];
  r2 = rmap[l2];
  s = l1;
  while(l1 < num){
    if(r1.color==r2.color && r1.color){
      if((x1>=x2 && x1<x2+r2.length) || (x2>=x1 && x2<x1+r1.length)){
        if(s != l1){
          rmap[l1].parent = r1.parent = r2.parent;
          s = l1;
        }else{
          // find terminal roots of each path
          n = r1.parent;
          while(n != rmap[n].parent) n = rmap[n].parent;
          p = r2.parent;
          while(p != rmap[p].parent) p = rmap[p].parent;

          // must use smaller of two to preserve DAGness!
          if(n < p){
            rmap[p].parent = n;
          }else{
            rmap[n].parent = p;
          }
        }
      }
    }

    // Move to next point where values may change
    if(x1+r1.length < x2+r2.length){
      x1 += r1.length;
      r1 = rmap[++l1];
    }else{
      x2 += r2.length;
      r2 = rmap[++l2];
    }
  }

  // Now we need to compress all parent paths
  for(i=0; i<num; i++){
    p = rmap[i].parent;
    if(p > i){
      while(p != rmap[p].parent) p = rmap[p].parent;
      rmap[i].parent = p;
    }else{
      rmap[i].parent = rmap[p].parent;
    }
  }
}

int CMVision::extractRegions(rle * rmap,int num)
// Takes the list of runs and formats them into a region table,
// gathering the various statistics we want along the way.
// num is the number of runs in the rmap array, and the number of
// unique regions in reg[] (< CMV_MAX_REGIONS) is returned.
// Implemented as a single pass over the array of runs.
{
  int x,y,i;
  int b,n,a;
  rle r;
  int width2=roi.x2-roi.x1+1;

  //x = y = n = 0;
  n=0;
  x=roi.x1;
  y=roi.y1;
  for(i=0; i<num; i++){
    r = rmap[i];

    if(r.color){
      if(r.parent == i){
        // Add new region if this run is a root (i.e. self parented)
        b = n;  // renumber to point to region id
		rmap[i].parent = n;
        blobs[b].color = r.color;//log2modp[r.color%37];//bottom_bit(r.color);
        blobs[b].area = r.length;
        blobs[b].x1 = x;
        blobs[b].y1 = y;
        blobs[b].x2 = x + r.length-1; //-1
        blobs[b].y2 = y;
        blobs[b].sum_x = range_sum(x,r.length);
        blobs[b].sum_y = y * r.length;
		blobs[b].start.x=x;
		blobs[b].start.y=y;
        n++;

		 //witek - wypelnianie obrazu etykiet - wolne!
		 for(int j=0;j<r.length;j++)
			labels[x+j+width*y]=n;

			//labels[x+j][y]=n;



        if(n >= CMV_MAX_REGIONS)
			return(CMV_MAX_REGIONS);
      }else{
        // Otherwise update region stats incrementally
        b = rmap[r.parent].parent;
        rmap[i].parent = b; // update to point to region id
        blobs[b].area += r.length;
        blobs[b].x2 = max(x + r.length-1,blobs[b].x2); //-1
        blobs[b].x1 = min(x,blobs[b].x1);
        blobs[b].y2 = y; // last set by lowest run
        blobs[b].sum_x += range_sum(x,r.length);
        blobs[b].sum_y += y * r.length;
		 //witek - wypelnianie obrazu etykiet - wolne!
	   	for(int j=0;j<r.length;j++)
			labels[x+j+width*y]=b+1;

			//labels[x+j][y]=b+1;


      }
    }

    // step to next location
    //x = (x + r.length) % width;
	x = x + r.length % width2;
	if(x==roi.x2+1)
		x=roi.x1;	//moje
    //y += (x == 0);

	y += (x == roi.x1);

  }


  // calculate centroids from stored temporaries
  for(i=0; i<n; i++){
    a = blobs[i].area;
    blobs[i].center.x = (float)blobs[i].sum_x / a;
    blobs[i].center.y = (float)blobs[i].sum_y / a;
  }

  blobs_count=n;
  return(n);
}


//==== Interface/Public Functions ==================================//

#define ZERO(x) memset(x,0,sizeof(x))

void CMVision::clear()
{
  ZERO(h_class);
  ZERO(s_class);
  ZERO(v_class);
  ZERO(colors);

  map = NULL;

}

bool CMVision::countLUT()
{
RGB2H[0x0]=0x0;
RGB2S[0x0]=0x0;
RGB2V[0x0]=0x0;

for(unsigned short int i=0x1; i<=0xfffe; i++)
{
unsigned char ir,ib,ig;
int oh,os,ov;

int maxrgb=0,minrgb=0;
ir=(i&0xf800)>>8;
ig=(i&0x07e0)>>3;
ib=(i&0x001f)<<3;

maxrgb=max3(ir,ig,ib);

minrgb=min3(ir,ig,ib);

					if(maxrgb==minrgb)
					{
						oh=0;
						os=0;
						ov=maxrgb;
					}
					else
					{
						ov=maxrgb;
						os=255*(maxrgb-minrgb)/maxrgb;
						if(ir==maxrgb)
							oh=60*(ig-ib)/(maxrgb-minrgb);
						else
							if(ig==maxrgb)
								oh=120+60*(ib-ir)/(maxrgb-minrgb);
							else
								oh=240+60*(ir-ig)/(maxrgb-minrgb);
					}

			//out[c].h*=60;
			if( oh < 0 )
				oh = (int)oh+360;
			else
				oh = (int)oh;

			oh=oh*255/360;

RGB2H[i]=(unsigned char)(oh+30);
RGB2S[i]=(unsigned char)(os);
RGB2V[i]=(unsigned char)(ov);

//printf("%d->%d\n",i,RGB2H[i]);
}

RGB2H[0xffff]=85;
RGB2S[0xffff]=4;
RGB2V[0xffff]=252;

return 0;
}

bool CMVision::initialize(int nwidth,int nheight)
// Initializes library to work with images of specified size
{
  width = nwidth;
  height = nheight;

  roi.x1=0;
  roi.x2=width-1;
  roi.y1=0;
  roi.y2=height-1;

  if(map) delete(map);

  map = new unsigned[width * height + 1];
  // Need 1 extra element to store terminator value in encodeRuns()

  if(labels) delete(labels);

 // labels = new unsigned int[width * height];

  labels = new unsigned[width * height + 1];

// ZERO(labels);

  return(map != NULL);
}

bool CMVision::initGrid()
{
	int korekta=360;

	static_middle_grid_xa[0]=173;
	static_middle_grid_xa[1]=207;
	static_middle_grid_xa[2]=242;
	static_middle_grid_xa[3]=138;
	static_middle_grid_xa[4]=173;
	static_middle_grid_xa[5]=209;
	static_middle_grid_xa[6]=105;
	static_middle_grid_xa[7]=137;
	static_middle_grid_xa[8]=173;

	static_middle_grid_ya[0]=72;
	static_middle_grid_ya[1]=107;
	static_middle_grid_ya[2]=142;
	static_middle_grid_ya[3]=107;
	static_middle_grid_ya[4]=142;
	static_middle_grid_ya[5]=178;
	static_middle_grid_ya[6]=142;
	static_middle_grid_ya[7]=178;
	static_middle_grid_ya[8]=214;

	if(korekta==360)
	for(int i=0; i<9; i++)
	{
		static_middle_grid_xa[i]+=12;
		static_middle_grid_ya[i]-=12;
	}

return 0;
}

bool CMVision::initEstim(const char *filename)
{
//w mroku vvector zamiast vector
	float x,y;

	cc=vvector(2);
	fc=vvector(2);
	kc=vvector(5);

/*
	fc[1]=751.860077541601300;
	fc[2]=757.240379484519850;

	cc[1]=368.297088758283450;
	cc[2]=275.241113833860310;

	kc[1]= -0.353305987532453;
	kc[2]= 0.224942921451107;
	kc[3]= 0.002144573630332;
	kc[4]= 0.000737975434375;
	kc[5]= 0.000000000000000;
*/
	fc[1]=1624.23566; //751.860077541601300;
	fc[2]=1630.87379; //757.240379484519850;

	cc[1]=378.16536; //368.297088758283450;
	cc[2]=266.82798; //275.241113833860310;

	//cc[1]=0; //368.297088758283450;
	//cc[2]=0; //275.241113833860310;

	kc[1]= 0.00481; //-0.353305987532453;
	kc[2]= 0.47232; //0.224942921451107;
	kc[3]= -0.00174; //0.002144573630332;
	kc[4]= -0.00569; //0.000737975434375;
	kc[5]= 0.000000000000000;

	x_kk=matrix(2,4);
	X_kk=matrix(3,4);

	omckk=vvector(3);
	Tckk=vvector(3);
	Rckk=matrix(3,3);


	alloc_m=0;
	alloc_v=0;



	X_kk[1][1]=-BETWEEN_CENTERS/2; X_kk[1][2]=BETWEEN_CENTERS/2; X_kk[1][3]=-BETWEEN_CENTERS/2; X_kk[1][4]=BETWEEN_CENTERS/2;
	X_kk[2][1]=BETWEEN_CENTERS/2; X_kk[2][2]=BETWEEN_CENTERS/2; X_kk[2][3]=-BETWEEN_CENTERS/2; X_kk[2][4]=-BETWEEN_CENTERS/2;
	X_kk[3][1]=-HALF_CUBE; X_kk[3][2]=-HALF_CUBE; X_kk[3][3]=-HALF_CUBE; X_kk[3][4]=-HALF_CUBE;


	pattern_fp = fopen( filename, "r" );


	for(int j=0; j<PATTERNSMAX; j++)
	{
		fscanf(pattern_fp,"%f %f ",&x,&y);
		whichpattern[j]=(int)(x);
  		tiles_count[j]=(int)(y);

	//	printf("%d -> \n",j);
		for(int i=0; i<tiles_count[j];i++)
		{
			fscanf(pattern_fp,"%f %f ",&x,&y);
			mappattern[j][i].x=x;
			mappattern[j][i].y=y;
			//printf("%f \n",eee);
		}
		//fscanf(pattern_fp,"\n");
	//	printf("\n");
	}

	fclose(pattern_fp);

	return 1;
}

void CMVision::freeEstim()
{
	free_matrix(Rckk);
	free_matrix(x_kk);
	free_matrix(X_kk);
	free_vector(cc);
	free_vector(fc);
	free_vector(kc);
	free_vector(omckk);
	free_vector(Tckk);
}

// sets bits in k in array arr[l..r]

void set_bits(unsigned int *arr,int len,int l,int r,unsigned int k)
{
  int i;

  l = max(l,0);
  r = min(r+1,len);

  for(i=l; i<r; i++) arr[i] |= k;
}


void clear_bits(unsigned int *arr,int len,int l,int r,unsigned int k)
{
  int i;

  l = max(l,0);
  r = min(r+1,len);

  k = ~k;
  for(i=l; i<r; i++) arr[i] &= k;
}

#define CMV_STATE_SCAN   0
#define CMV_STATE_COLORS 1
#define CMV_STATE_THRESH 2
#define CMV_MAX_BUF 256

bool CMVision::loadColors(const char *filename)
// Loads in colors file specifying color names and representative
// rgb triplets.  Also loads in color class threshold values.
{
  char buf[CMV_MAX_BUF],str[CMV_MAX_BUF];
  FILE *in;
  int state,i,n;

  int r,g,b;
  int class_num;
  color_info *c;

  int h1,h2,s1,s2,v1,v2;
  unsigned k;

  // Open colors file
  in = fopen(filename,"rt");
  if(!in) return(false);

  // Clear out previously set colors
  for(i=0; i<CMV_COLOR_LEVELS; i++){
    h_class[i] = s_class[i] = v_class[i] = 0;
  }
  for(i=0; i<CMV_MAX_COLORS; i++){
    if(colors[i].name){
      delete(colors[i].name);
      colors[i].name = NULL;
    }
  }

  // Loop ever lines, processing via a simple parser
  state = 0;
  while(fgets(buf,CMV_MAX_BUF,in)){
    switch(state){
      case CMV_STATE_SCAN:
        n = sscanf(buf,"[%s",str);
        if(n == 1){
          if(!strncmp(str,"colors]",CMV_MAX_BUF)){
            state = CMV_STATE_COLORS;
            i = 0;
	  }else if(!strncmp(str,"thresholds]",CMV_MAX_BUF)){
	    state = CMV_STATE_THRESH;
            i = 0;
	  }else{
            printf("CMVision: Ignoring unknown option header '%s'.\n",str);
          }
	}
        break;
      case CMV_STATE_COLORS:
        n = sscanf(buf,"(%d,%d,%d) %d %s",&r,&g,&b,&class_num,str);
        if(n == 5){
          if(i < CMV_MAX_COLORS){
            c = &colors[i];
            c->color.r   = r;
            c->color.g = g;
            c->color.b  = b;
            c->name  = strdup(str);
            c->class_num = class_num;
            i++;
	  }else{
	    printf("CMVision: Too many colors, ignoring '%s'.\n",str);
	  }
	}else if(n == 0){
          state = CMV_STATE_SCAN;
        }
        break;
      case CMV_STATE_THRESH:
        n = sscanf(buf,"(%d:%d,%d:%d,%d:%d)",&h1,&h2,&s1,&s2,&v1,&v2);
        if(n == 6){
          if(i < CMV_MAX_COLORS){
            c = &colors[i];
            c->h_low = h1;  c->h_high = h2;
            c->s_low = s1;  c->s_high = s2;
            c->v_low = v1;  c->v_high = v2;

            k = (1 << i); //kolejne zapalone bity oznaczaj� kolejne kolory

			set_bits(h_class,CMV_COLOR_LEVELS,h1,h2,k);
            set_bits(s_class,CMV_COLOR_LEVELS,s1,s2,k);
            set_bits(v_class,CMV_COLOR_LEVELS,v1,v2,k);
			color_tab[k % 37]=c->class_num; //witek - laczenie blobow o roznych kolorach
            i++;
	  }else{
	    printf("CMVision: Too many thresholds.\n");
	  }
	}else if(n == 0){
          state = CMV_STATE_SCAN;
        }
        break;
    }
  }


  fclose(in);

  return(true);
}


void CMVision::close()
{
  if(map) delete(map);
  map = NULL;

	free_matrix(Rckk);
	free_matrix(x_kk);
	free_matrix(X_kk);
	free_vector(cc);
	free_vector(fc);
	free_vector(kc);
	free_vector(omckk);
	free_vector(Tckk);

//	free_vector(cube_vector);
// 	free_vector(cube_center);
// 	free_vector(cube_temp);
}


//==== Main Vision Functions =======================================//

//bool CMVision::findBlobs(image_pixel *image)
bool CMVision::findBlobs(unsigned short * buffer)
{
  int runs;

  if(!buffer) return(false);

  classifyFrame(buffer,map);
  runs = encodeRuns(rmap,map);
  connectComponents(rmap,runs);
  extractRegions(rmap,runs);
  return(true);
}

/*
bool CMVision::findBlobsfilter(image_pixel *image, filter data)
{
	findBlobs(image);

	filterBlobsReset();

	filterBlobs(BLOB_SIZE_BIGGER, data.area_min);
	filterBlobs(BLOB_SIZE_SMALLER, data.area_max);
	findVerticesAll();
	filterBlobs(CONTOUR_LENGTH_BIGGER, data.contour_min);
	filterBlobs(CONTOUR_LENGTH_SMALLER, data.contour_max);
	filterBlobs(VERTICES_BIGGER, data.vertices_min);
	filterBlobs(VERTICES_SMALLER, data.vertices_max);
	filterBlobs(BLOB_CIRCULARITY_BIGGER, data.circularity_min);
	filterBlobs(BLOB_CIRCULARITY_SMALLER, data.circularity_max);
	filterBlobs(NEIGHBORS, data.neighbors);
	return true;
}
*/

unsigned char CMVision::contour_point(int x, int y, unsigned int nr) //nr labels wokol ktorej szukamy krawedzi
		{

			if(x<roi.x1 || x>roi.x2 || y<roi.y1 || y>roi.y2) return 0;
			if(map[x+y*width]==nr)
			{
				if(x==roi.x1 || y==roi.y1) goto jest;
				if(y>roi.y1 && map[x+(y-1)*width]!=nr) goto jest;

				if(y<roi.y2 && map[x+(y+1)*width]!=nr) goto jest;

				if(x>roi.x1 && map[x-1+y*width]!=nr) goto jest;

				if(x<roi.x2 && map[x+1+y*width]!=nr) goto jest;

				if(y>roi.y1 && x>roi.x1 && map[x-1+(y-1)*width]!=nr) goto jest;
				if(y>roi.y1 && x<roi.x2 && map[x+1+(y-1)*width]!=nr) goto jest;
				if(y<roi.y2 && x<roi.x2 && map[x+1+(y+1)*width]!=nr) goto jest;
				if(y<roi.y2 && x>roi.x1 && map[x-1+(y+1)*width]!=nr) goto jest;
				return 0;

			jest:
				//jesli jakikolwiek punkt wokol danego byl pusty, to znaczy ze dany
				//punkt nalezy do krawedzi (nie uwzglednia to dziur!)
				contour.length++;
				contour.points[contour.length].x=x;
				contour.points[contour.length].y=y;
				return 1;

			}
			return 0;
		}

int CMVision::findContour(int blob_nr)
		{

			//pierwszy punkt krawedzi jest w	bloby.startx[k] i bloby.starty[k]
			short x=blobs[blob_nr].start.x;
			short y=blobs[blob_nr].start.y;

			if(blobs[blob_nr].area==1) //zabezpieczenie przed blobami o rozmiarze 1 piksela
			{
				contour.points[0].x=x;
				contour.points[0].y=y;
				contour.length=1;
				return(1);
			}

			int k=map[x+y*width]; //kolor pierwszego punktu bloba
			int kierunek=2; //0-N, 1-W, 2-S, 3-E
			//zawsze sprawdzamy punkty z przodu i z prawej wzgledem kierunku
			unsigned char jest=0, koniec=0;


			contour.length=0;
			contour.points[contour.length].x=x;
			contour.points[contour.length].y=y;

			do
			{	//sprawdzamy 2 punkty;  po prawej i na wprost

				// punkt z prawej
				if(kierunek==2)	jest=contour_point(contour.points[contour.length].x-1,contour.points[contour.length].y,k);
				else
					if(kierunek==3)	jest=contour_point(contour.points[contour.length].x,contour.points[contour.length].y+1,k);
				else
					if(kierunek==0)	jest=contour_point(contour.points[contour.length].x+1,contour.points[contour.length].y,k);
				else
					if(kierunek==1)	jest=contour_point(contour.points[contour.length].x,contour.points[contour.length].y-1,k);

				if(jest==1) //znaleziono punkt z prawej wiec nastepuje zmiana kierunku
				{
					//zmiana kierunku z S na W
					if(kierunek==2) {kierunek=1; jest=1;}
					else
						//zmiana kierunku z E na S
						if(kierunek==3) {kierunek=2; jest=1;}
						//zmiana kierunku z N na E
					else
						if(kierunek==0) {kierunek=3; jest=1;}
						//zmiana kierunku z W na N
					else
						if(kierunek==1) {kierunek=0; jest=1;}
				}
				// punkt na wprost
				if(jest==0)
					if(kierunek==2)	jest=contour_point(contour.points[contour.length].x,contour.points[contour.length].y+1,k);
				else
					if(kierunek==3)	jest=contour_point(contour.points[contour.length].x+1,contour.points[contour.length].y,k);
				else
					if(kierunek==0)	jest=contour_point(contour.points[contour.length].x,contour.points[contour.length].y-1,k);
				else
					if(kierunek==1)	jest=contour_point(contour.points[contour.length].x-1,contour.points[contour.length].y,k);


				//jesli nie ma nowego punktu zmieniamy kierunek poszukiwan o 90 st w lewo
				if(jest==0)
				{
					//zmiana kierunku z S na E
					if(kierunek==2) {kierunek=3; jest=1;}
					else
						//zmiana kierunku z E na N
						if(kierunek==3) {kierunek=0; jest=1;}
						//zmiana kierunku z N na W
					else
						if(kierunek==0) {kierunek=1; jest=1;}
						//zmiana kierunku z W na S
					else
						if(kierunek==1) {kierunek=2; jest=1;}
				}
				//sprawdzanie 2 punktow od biezacego wstecz
				//jesli biezacy punkt to ten sam co punkt nr 1 to
				if(contour.length>1)
					if(contour.points[contour.length].x==contour.points[1].x && contour.points[contour.length].y==contour.points[1].y)
					//jesli dodatkowo poprzedni punkt to punkt startowy to KONIEC
						if(contour.points[contour.length-1].x==contour.points[0].x && contour.points[contour.length-1].y==contour.points[0].y)
							koniec=1;
						else
							koniec=0;


			}while(jest==1 && (!koniec || contour.length==0));
		contour.length-=1; //zmniejszamy o 1 bo zamykajac petle doszlismy
					 //az do pierwszego punktu (zerowy i pierwszy)
					//ale nie zmniejszamy o 2 bo punkty liczymy od zerowego

		//dopisanie parametrow krawedzi do bloba
		blobs[blob_nr].contour_length=contour.length;
		return(contour.length);

		}


void CMVision::znajdz_najdalszy(int p1, int p2, int kwadrat_odl, unsigned char wszystkie)
		{
			double a=0,b=0,b2=0,tmp=0,x1,y1,x2,y2,odleglosc=0;
			unsigned char pion=0, poziom=0, wstecz=0;
			int i,najdalszy=0,p2d; //p2d - dodatkowy punkt zeby nie zamazac oryginalnego

	p2d=p2;
			//wyznaczenie rownania prostej przechadzacej przez punkty o numerach: l1, l2


			if(contour.points[p1].x==contour.points[p2].x) pion=1;	// prosta pionowa
			if(contour.points[p1].y==contour.points[p2].y) poziom=1;	// prosta pozioma


			if(pion==0 && poziom==0)
			{
				a=(double)(contour.points[p1].y-contour.points[p2].y)/(double)(contour.points[p1].x-contour.points[p2].x);
				b=(double)contour.points[p1].y-(double)a*contour.points[p1].x;
			}

			//wyznaczenie najwiekszej odleglosci punktu segmentu od prostej

			//odleglosc=0; //najwiekszy kwadrat odleglosci prostej od punktu


			if(p1>p2) //przeszukiwanie drugiej czesci figury od prawego do lewego
			{
				wstecz=1;
				// ustawienie poszukiwania od prawego do konca
				p2d=contour.length-1; //pierwszy ounkt  zostaje bez zmian, a drugi jest ostatnim punktem obwodu
			}
			for(i=p1;i<=p2d;i++)
			{
				if(pion==1)
				{
					x1=contour.points[i].x;   // punkt w poblizu prostej
					//y1=ky[i];
					tmp=(x1-contour.points[p1].x)*(x1-contour.points[p1].x);
				}
				else
					if(poziom==1)
				{
					//x1=kx[i];   // punkt w poblizu prostej
					y1=contour.points[i].y;
					tmp=(y1-contour.points[p1].y)*(y1-contour.points[p1].y);
				}
				else  //prosta skosna
				{
					x1=contour.points[i].x;   // punkt, ktorego odleglosc mierzymy
					y1=contour.points[i].y;

					b2=y1+x1/a;

					x2=(b2-b)/(a+1.0/a);	// punkt "prostopadly"
					y2=a*x2+b;

					tmp=(x1-x2)*(x1-x2)+(y1-y2)*(y1-y2); //kwadrat odleglosci
				}
				if(tmp>=odleglosc)
				{
					odleglosc=tmp;
					najdalszy=i;
				}
			}

			if(wstecz==1) //jesli nie zaczynamy(konczymy) na zerowym punkcie zostalo nam kilka punktow
				for(i=0;i<p2;i++)
				{
					if(pion==1)
					{
						x1=contour.points[i].x;   // punkt w poblizu prostej
						//y1=ky[i];
						tmp=(x1-contour.points[0].x)*(x1-contour.points[0].x);
					}
					else
						if(poziom==1)
					{
						//x1=kx[i];   // punkt w poblizu prostej
						y1=contour.points[i].y;
						tmp=(y1-contour.points[0].y)*(y1-contour.points[0].y);
					}
					else  //prosta skosna
					{
						x1=contour.points[i].x;   // punkt, ktorego odleglosc mierzymy
						y1=contour.points[i].y;

						b2=y1+x1/a;

						x2=(b2-b)/(a+1.0/a);	// punkt "prostopadly"
						y2=a*x2+b;

						tmp=(x1-x2)*(x1-x2)+(y1-y2)*(y1-y2); //kwadrat odleglosci
					}


					if(tmp>=odleglosc)
					{
						odleglosc=tmp;
						najdalszy=i;
					}
				}


			if(odleglosc>kwadrat_odl) //znaczy, ze trzeba dokonac kolejnego podzialu
			{
				(vertices.count)++;	//zwiekszenie licznika wierzcholkow
				vertices.points[vertices.count-1]=najdalszy;	//wpisanie nowego wierzcholka
				if(wszystkie==1)
				{
					znajdz_najdalszy(p1,najdalszy,kwadrat_odl,1);	//sprawdzenie z lewej
					znajdz_najdalszy(najdalszy,p2,kwadrat_odl,1); //sprawdzenie z prawej
				}
			}
			else	//czasami przy podziale wstepnym nie trzeba dzielic niemniej nalezy wyznaczyc jakis punkt
				if(wszystkie==0 && wstecz==0)
				{
				vertices.count++;	//zwiekszenie licznika wierzcholkow
				if((najdalszy-vertices.points[0])>(vertices.points[1]-najdalszy))
					vertices.points[vertices.count-1]=vertices.points[1];
				else
					vertices.points[vertices.count-1]=vertices.points[0];
				}
			else
			if(wszystkie==0 && wstecz==1)
			{
				vertices.count++;	//zwiekszenie licznika wierzcholkow
				if((vertices.points[0]+contour.length-najdalszy)>(najdalszy-vertices.points[1]))
					vertices.points[vertices.count-1]=vertices.points[1];
				else
					vertices.points[vertices.count-1]=vertices.points[0];
			}

		}

void CMVision::findVertices(int dokl)
		{
			unsigned int i,j,tmp;
			int lewy=0, prawy=0;


			//znalezienie skrajnych punktow z lewej i prawej

				for(i=0;i<contour.length;i++)
					if(contour.points[i].x<contour.points[lewy].x) lewy=i;
					else
					if(contour.points[i].x>contour.points[prawy].x) prawy=i;


				//zapisanie pierwszych dwoch wierzcholkow
				vertices.count=2;		//liczba wierzcholkow
				vertices.points[0]=lewy;
				vertices.points[1]=prawy;

				for(int ile=0;ile<3;ile++)
				{
				//teraz wyznaczymy dwa najdalej polozone punkty w dol i w gore od prostej laczacej
				//dwa wstepne skrajne punkty
				znajdz_najdalszy(lewy,prawy,1,0);
				znajdz_najdalszy(prawy,lewy,1,0);
				//te dwa nowe punkty beda naszymi wyjsciowymi punktami do poszukiwania aproksymacji
				vertices.count=2;
				vertices.points[0]=vertices.points[2];
				vertices.points[1]=vertices.points[3];
				lewy=vertices.points[0];
				prawy=vertices.points[1];

// ten fragment powtorzony dla lepszego znalezienie skrajnych punktow
			znajdz_najdalszy(lewy,prawy,1,0);
			znajdz_najdalszy(prawy,lewy,1,0);
			//te dwa nowe punkty beda naszymi wyjsciowymi punktami do poszukiwania aproksymacji
			vertices.count=2;
			vertices.points[1]=vertices.points[2]; //zamiana kolejnosci
			vertices.points[0]=vertices.points[3];
			lewy=vertices.points[0];
			prawy=vertices.points[1];
		}

			znajdz_najdalszy(lewy,prawy,dokl,1);
			znajdz_najdalszy(prawy,lewy,dokl,1);

				//sortowanie wierzcholkow
				for(i=0;i<vertices.count;i++)
				{
					tmp=vertices.points[i];
					j = i;
					while ((j>0) && (tmp<vertices.points[j-1]))
					{
						vertices.points[j] = vertices.points[j-1];
						j--;
					}
					vertices.points[j] = tmp;
				}
			}
int CMVision::findVertices(int dokl, int blob_nr)
{

	if(dokl==AUTO)
	{
		double x=blobs[blob_nr].contour_length;
		dokl=(int)(0.00001*x*x*x-0.003375*x*x+0.518*x-10);
		//dokl=(int)(0.00001*x*x*x-0.005375*x*x+0.518*x-10);
		if(dokl<1) dokl=1;

	}

	findVertices(dokl);
	return(dokl);
}

void CMVision::findVerticesAll(void)
{

			for(unsigned int i=0;i<filtered_blobs_count;i++)
				{
				findContour(filtered_blobs[i]);
				findVertices(AUTO, filtered_blobs[i]);
				blobs[filtered_blobs[i]].vertices_count=vertices.count;
				if(vertices.count<=10)
					for(unsigned int j=0;j<vertices.count;j++)
					{
						blobs[filtered_blobs[i]].vertices[j].x=contour.points[vertices.points[j]].x;
						blobs[filtered_blobs[i]].vertices[j].y=contour.points[vertices.points[j]].y;
					}
				}
}


void CMVision::filterBlobsReset(void)
{
	/*	Funkcja ustawia wektor numerow blobow (ktore nas interesuja) filtered_blobs
		tak by zawieral wszystkie wykryte w obrazie bloby

		Wejscie:
		blobs - tablica struktur przechowujaca parametry blobow

		Wyjscie:
		filtered_blobs - wektor zawierajacy numery interesujacych blobow
						(w tym wypadku WSZYSTKICH blobow)
		UWAGI:
		blobs_count - zmienna globalna przechowujaca liczbe wszystkich wykrytych
						w obrazie blobow
	*/
	for(unsigned int i=0;i<blobs_count;i++)
		filtered_blobs[i]=i;
	filtered_blobs_count=blobs_count;
}
bool CMVision::filterBlobs(int filter, double value)
{
	/*	Funkcja dokonuje filtracji wektora numerow blobow (ktore nas interesuja) filtered_blobs
		wg podanego kryterium i wartosci

		Wejscie:
		blob - tablica struktur przechowujaca parametry blobow
		filtered_blobs - wektor aktualnych blobow
		filter - nazwa filtra (zdefiniowane na poczatku pliku)
		value - wartosc

		Wyjscie:
		filtered_blobs - wektor zawierajacy numery interesujacych blobow
						(w tym wypadku WSZYSTKICH blobow)
		UWAGI:
		blobs_count - zmienna globalna przechowujaca liczbe wszystkich wykrytych
						w obrazie blobow
	*/
	if(value==0) return false;
	int starting_size=filtered_blobs_count; //tyle jest blobow w wektorze przed filtracja
	filtered_blobs_count=0;
	if(filter==BLOB_SIZE_BIGGER)
	{
		for(int i=0;i<starting_size;i++)
			if(blobs[filtered_blobs[i]].area>=value)
				filtered_blobs[filtered_blobs_count++]=filtered_blobs[i];


	}
	else
	if(filter==BLOB_SIZE_SMALLER)
	{
		for(int i=0;i<starting_size;i++)
			if(blobs[filtered_blobs[i]].area<=value)
				filtered_blobs[filtered_blobs_count++]=filtered_blobs[i];

	}
	else
	if(filter==BLOB_CIRCULARITY_BIGGER) //idealne kolo ma wskaznik ok. 0.8 w naszym przypadku
	{									//szesciokat foremny 0.9
										//kwadrat z zaokraglonymi rogami 1.1
										//kwadrat z ostrymi rogami 1.2
										//trojkat rownoramienny 1.3
										//gwiazda 2.5-3.0
										//pierscien (stosunek srednic 0.66) 1.5

		for(int i=0;i<starting_size;i++)
			if(blobs[filtered_blobs[i]].contour_length*blobs[filtered_blobs[i]].contour_length*0.25*M_1_PI/(double)(blobs[filtered_blobs[i]].area)>=value)
				filtered_blobs[filtered_blobs_count++]=filtered_blobs[i];

	}
	else
	if(filter==BLOB_CIRCULARITY_SMALLER)
	{
		for(int i=0;i<starting_size;i++)
			if(blobs[filtered_blobs[i]].contour_length*blobs[filtered_blobs[i]].contour_length*0.25*M_1_PI/(double)(blobs[filtered_blobs[i]].area)<=value)
				filtered_blobs[filtered_blobs_count++]=filtered_blobs[i];

	}
	else
	if(filter==CONTOUR_LENGTH_BIGGER)
	{
		for(int i=0;i<starting_size;i++)
			if(blobs[filtered_blobs[i]].contour_length>=value)
				filtered_blobs[filtered_blobs_count++]=filtered_blobs[i];

	}
	else
	if(filter==CONTOUR_LENGTH_SMALLER)
	{
		for(int i=0;i<starting_size;i++)
			if(blobs[filtered_blobs[i]].contour_length<=value)
				filtered_blobs[filtered_blobs_count++]=filtered_blobs[i];

	}
	else
	if(filter==VERTICES_BIGGER)
	{
		for(int i=0;i<starting_size;i++)
			if(blobs[filtered_blobs[i]].vertices_count>=value)
				filtered_blobs[filtered_blobs_count++]=filtered_blobs[i];

	}
	else
	if(filter==VERTICES_SMALLER)
	{
		for(int i=0;i<starting_size;i++)
			if(blobs[filtered_blobs[i]].vertices_count<=value)
				filtered_blobs[filtered_blobs_count++]=filtered_blobs[i];

	}

	else
	if(filter==NEIGHBORS)
	{
		//odrzuca te bloby, ktore maja mniej niz number sasiadow o podobnej wielkosci w poblizu.
		int distance_sq; //kwadrat odleglosci - nie ma potrzeby pierwiastkowac
		int tmp[1000]; //tymczasowa kopia wektora blobow
		int t;
		tmp[0]=0;

		for(int j=1;j<=starting_size;j++)
		{
		int x_1=(int)blobs[filtered_blobs[j]].center.x; //wsp. bloba dla ktorego
		int y_1=(int)blobs[filtered_blobs[j]].center.y; //szukamy podobnych sasiadow

		t=0;
		for(int i=0;i<starting_size;i++)
		{
			int x=(int)blobs[filtered_blobs[i]].center.x;
			int y=(int)blobs[filtered_blobs[i]].center.y;
			distance_sq=(x_1-x)*(x_1-x)+(y_1-y)*(y_1-y);
			//	Promien poszukiwania "radius" jest wyznaczany na podstawie prostokata otaczajacego bloba

			int radius_sq=(int)1.5*(blobs[filtered_blobs[i]].x2-blobs[filtered_blobs[i]].x1)*(blobs[filtered_blobs[i]].x2-blobs[filtered_blobs[i]].x1)+(blobs[filtered_blobs[i]].y2-blobs[filtered_blobs[i]].y1)*(blobs[filtered_blobs[i]].y2-blobs[filtered_blobs[i]].y1);
			if(distance_sq<radius_sq)
				if(blobs[filtered_blobs[i]].area<2*blobs[filtered_blobs[j]].area && blobs[filtered_blobs[i]].area>0.5*blobs[filtered_blobs[j]].area)
					t++;
		}
			if(t>=value+1)
			{
				tmp[0]++;
				tmp[tmp[0]]=filtered_blobs[j];
			}
		}
		for(int i=0;i<tmp[0];i++)
			filtered_blobs[i]=tmp[i+1];

		filtered_blobs_count=tmp[0];

	}
return true;
}



void CMVision::setRoi(roi_rubik r, int m){


	roi.x1=r.x1-m;
	roi.y1=r.y1-m;
	roi.x2=r.x2+m;
	roi.y2=r.y2+m;
	if(roi.x1<0) roi.x1=0;
	if(roi.y1<0) roi.y1=0;
	if(roi.x2>width-1) roi.x2=width-1;
	if(roi.y2>height-1) roi.y2=height-1;


}


bool CMVision::estimPose()
{


	compute_extrinsic_init(x_kk,X_kk,omckk,Tckk,Rckk,fc,cc,kc);

	compute_extrinsic_refine(x_kk,X_kk,omckk,Tckk,Rckk,fc,cc,kc);


	return 1;
}

bool CMVision::objectClass()
{
	goal_meanx=0;
	goal_meany=0;

	endeffector_meanx=0;
	endeffector_meany=0;
	goal_blobs_count=0;
	endeffector_blobs_count=0;

	if(filtered_blobs_count<3)
	{
		whole_face=0;
		return 1;
	}

	for(unsigned int i=0;i<blobs_count;i++)
		valid_blobs[i]=0;

	for(unsigned int i=0;i<blobs_count;i++)
		goal_blobs[i]=0;
	for(unsigned int i=0;i<blobs_count;i++)
		endeffector_blobs[i]=0;

	for (unsigned int i=0; i<filtered_blobs_count; i++)
	{
			goal_meanx+=(blobs[filtered_blobs[i]].color!=6)*blobs[filtered_blobs[i]].center.x;
			goal_meany+=(blobs[filtered_blobs[i]].color!=6)*blobs[filtered_blobs[i]].center.y;
			endeffector_meanx+=(blobs[filtered_blobs[i]].color==6)*blobs[filtered_blobs[i]].center.x;
			endeffector_meany+=(blobs[filtered_blobs[i]].color==6)*blobs[filtered_blobs[i]].center.y;
			valid_blobs[filtered_blobs[i]]=1;
			goal_blobs[filtered_blobs[i]]=(blobs[filtered_blobs[i]].color==6);
			endeffector_blobs[filtered_blobs[i]]=(blobs[filtered_blobs[i]].color==6);
			goal_blobs_count+=(blobs[filtered_blobs[i]].color!=6);
			endeffector_blobs_count+=(blobs[filtered_blobs[i]].color==6);
		//	printf("@@@@@@@@@@@@@ %d %f %f\n",blobs[filtered_blobs[i]].color, meanx, meany);
	}

	goal_meanx/=goal_blobs_count;
	goal_meany/=goal_blobs_count;
	endeffector_meanx/=endeffector_blobs_count;
	endeffector_meany/=endeffector_blobs_count;


}

bool CMVision::estimPose2(int is_effector, double meanx, double meany, double &matched_Z, int &matched_pattern, point_d &ff1, point_d &ff2, point_d &ff3, point_d &ff4)
{
//printf("est0\n");
	double n;
	double n_dist_min=0;
	int blob_nr=0;
	double dist_acc=0;
	double dist_min=0;
	int dist_argmin=0;
	int matched_regions_count=0;
	int matched_for_min=0;
	unsigned char unchecked_blobs[CMV_MAX_REGIONS];

	int nr_in_pattern;



	dist_min=1000000;


	for(int j=0; j<PATTERNSMAX; j++)
	{
		for(int m=-5; m<=2; m++)
		{
			n=1-(double)m/10;
			for(int l=0; l<=0; l++)
			{
				for(int k=0; k<=0; k++)
				{
					for (unsigned int ii=0; ii<filtered_blobs_count; ii++)
					{
						unchecked_blobs[filtered_blobs[ii]]=1;
					}
					dist_acc=0;
					matched_regions_count=0;

					if(!is_effector)
					{
						for(int i=0; i<tiles_count[j];i++)
						{
							blob_nr=labels[(unsigned)(mappattern[j][i].x*n+meanx+k)+width*(unsigned)(mappattern[j][i].y*n+meany+l)]-1;

							//goal
							dist_acc+=
							(blobs[blob_nr].color!=6)*unchecked_blobs[blob_nr]*valid_blobs[blob_nr]*
							((blobs[blob_nr].center.x-mappattern[j][i].x*n-meanx-k)*(blobs[blob_nr].center.x-mappattern[j][i].x*n-meanx-k)+
							(blobs[blob_nr].center.y-mappattern[j][i].y*n-meany-l)*(blobs[blob_nr].center.y-mappattern[j][i].y*n-meany-l));

							matched_regions_count+=(blobs[blob_nr].color!=6)*unchecked_blobs[blob_nr]*valid_blobs[blob_nr];

							unchecked_blobs[blob_nr]=0;
						}
					}

					if(is_effector)
					{
					for(int i=0; i<=8;i+=2) //for 0, 2 6, 8 - corners from 9 points pattern
						{
							blob_nr=labels[(unsigned)(mappattern[j][i].x*n+meanx+k)+width*(unsigned)(mappattern[j][i].y*n+meany+l)]-1;

							//goal
							dist_acc+=
							(blobs[blob_nr].color==6)*unchecked_blobs[blob_nr]*valid_blobs[blob_nr]*
							((blobs[blob_nr].center.x-mappattern[j][i].x*n-meanx-k)*(blobs[blob_nr].center.x-mappattern[j][i].x*n-meanx-k)+
							(blobs[blob_nr].center.y-mappattern[j][i].y*n-meany-l)*(blobs[blob_nr].center.y-mappattern[j][i].y*n-meany-l));

							matched_regions_count+=(blobs[blob_nr].color==6)*unchecked_blobs[blob_nr]*valid_blobs[blob_nr];

							unchecked_blobs[blob_nr]=0;

							i+=(2*(i==2));

						}
					}

					//goal
					dist_acc/=matched_regions_count;

					if(!is_effector)
					{
						if( matched_regions_count<=5)
						{
							dist_acc=1000000;
						}
					}

					if(is_effector)
					{
						if( matched_regions_count<=3)
						{
							dist_acc=1000000;
						}
					}
					dist_acc-=matched_regions_count*matched_regions_count;
					/*
					if(j==0)
						printf("SSS 0 %f \n", dist_acc);

					if(j==8)
						printf("SSS 8 %f \n", dist_acc);

					if(j==9)
						printf("SSS 9 %f \n", dist_acc);

					if(j==10)
						printf("SSS 10 %f \n", dist_acc);
					*/

					if(dist_acc<dist_min)
					{
						dist_argmin=j;
						dist_min=dist_acc;
						n_dist_min=n;
						matched_for_min=matched_regions_count;

						//4 narozne
						if(dist_argmin%12<=6)
						{
							blob_nr=labels[(unsigned)(mappattern[j][8].x*n+meanx+k)+width*(unsigned)(mappattern[j][8].y*n+meany+l)]-1;
							ff4.x=valid_blobs[blob_nr]*blobs[blob_nr].center.x+(!valid_blobs[blob_nr])*(mappattern[j][8].x*n+meanx+k);
							ff4.y=valid_blobs[blob_nr]*blobs[blob_nr].center.y+(!valid_blobs[blob_nr])*(mappattern[j][8].y*n+meany+l);

							blob_nr=labels[(unsigned)(mappattern[j][6].x*n+meanx+k)+width*(unsigned)(mappattern[j][6].y*n+meany+l)]-1;
							ff3.x=valid_blobs[blob_nr]*blobs[blob_nr].center.x+(!valid_blobs[blob_nr])*(mappattern[j][6].x*n+meanx+k);
							ff3.y=valid_blobs[blob_nr]*blobs[blob_nr].center.y+(!valid_blobs[blob_nr])*(mappattern[j][6].y*n+meany+l);

							blob_nr=labels[(unsigned)(mappattern[j][2].x*n+meanx+k)+width*(unsigned)(mappattern[j][2].y*n+meany+l)]-1;
							ff2.x=valid_blobs[blob_nr]*blobs[blob_nr].center.x+(!valid_blobs[blob_nr])*(mappattern[j][2].x*n+meanx+k);
							ff2.y=valid_blobs[blob_nr]*blobs[blob_nr].center.y+(!valid_blobs[blob_nr])*(mappattern[j][2].y*n+meany+l);

							blob_nr=labels[(unsigned)(mappattern[j][0].x*n+meanx+k)+width*(unsigned)(mappattern[j][0].y*n+meany+l)]-1;
							ff1.x=valid_blobs[blob_nr]*blobs[blob_nr].center.x+(!valid_blobs[blob_nr])*(mappattern[j][0].x*n+meanx+k);
							ff1.y=valid_blobs[blob_nr]*blobs[blob_nr].center.y+(!valid_blobs[blob_nr])*(mappattern[j][0].y*n+meany+l);
						}
						else
						{
							nr_in_pattern=2;
							blob_nr=labels[(unsigned)(mappattern[j][nr_in_pattern].x*n+meanx+k)+width*(unsigned)(mappattern[j][nr_in_pattern].y*n+meany+l)]-1;
							ff4.x=valid_blobs[blob_nr]*blobs[blob_nr].center.x+(!valid_blobs[blob_nr])*(mappattern[j][nr_in_pattern].x*n+meanx+k);
							ff4.y=valid_blobs[blob_nr]*blobs[blob_nr].center.y+(!valid_blobs[blob_nr])*(mappattern[j][nr_in_pattern].y*n+meany+l);
							nr_in_pattern=8;
							blob_nr=labels[(unsigned)(mappattern[j][nr_in_pattern].x*n+meanx+k)+width*(unsigned)(mappattern[j][nr_in_pattern].y*n+meany+l)]-1;
							ff3.x=valid_blobs[blob_nr]*blobs[blob_nr].center.x+(!valid_blobs[blob_nr])*(mappattern[j][nr_in_pattern].x*n+meanx+k);
							ff3.y=valid_blobs[blob_nr]*blobs[blob_nr].center.y+(!valid_blobs[blob_nr])*(mappattern[j][nr_in_pattern].y*n+meany+l);
							nr_in_pattern=0;
							blob_nr=labels[(unsigned)(mappattern[j][nr_in_pattern].x*n+meanx+k)+width*(unsigned)(mappattern[j][nr_in_pattern].y*n+meany+l)]-1;
							ff2.x=valid_blobs[blob_nr]*blobs[blob_nr].center.x+(!valid_blobs[blob_nr])*(mappattern[j][nr_in_pattern].x*n+meanx+k);
							ff2.y=valid_blobs[blob_nr]*blobs[blob_nr].center.y+(!valid_blobs[blob_nr])*(mappattern[j][nr_in_pattern].y*n+meany+l);
							nr_in_pattern=6;
							blob_nr=labels[(unsigned)(mappattern[j][nr_in_pattern].x*n+meanx+k)+width*(unsigned)(mappattern[j][nr_in_pattern].y*n+meany+l)]-1;
							ff1.x=valid_blobs[blob_nr]*blobs[blob_nr].center.x+(!valid_blobs[blob_nr])*(mappattern[j][nr_in_pattern].x*n+meanx+k);
							ff1.y=valid_blobs[blob_nr]*blobs[blob_nr].center.y+(!valid_blobs[blob_nr])*(mappattern[j][nr_in_pattern].y*n+meany+l);
						}




					}



			}

		}



		}
	}



//	x_kk[1][1]=a2.x; x_kk[1][2]=a3.x; x_kk[1][4]=a5.x; x_kk[1][3]=a4.x;
//	x_kk[2][1]=a2.y; x_kk[2][2]=a3.y; x_kk[2][4]=a5.y; x_kk[2][3]=a4.y;

	matched_pattern=dist_argmin;
	matched_Z=n_dist_min;
	whole_face=1;

/*
	printf("XXXXXXXX %d, %f\n", matched_pattern, matched_Z);

						cout << "FUN" << endl;
						cout << ff1.x<<"; " << ff2.x<<"; " << ff3.x<<"; " << ff4.x << endl;
						cout << ff1.y<<"; " << ff2.y<<"; " << ff3.y<<"; " << ff4.y << endl;
*/
	return 1;
}

bool CMVision::estimPose3()
{
	objectClass();

	//0 - goal, 1 - endeffector
	estimPose2(0, goal_meanx, goal_meany, goal_matched_Z, goal_matched_pattern, goal_f1, goal_f2, goal_f3, goal_f4);
	estimPose2(1, endeffector_meanx, endeffector_meany, endeffector_matched_Z, endeffector_matched_pattern, endeffector_f1, endeffector_f2, endeffector_f3, endeffector_f4);

	//Image Jacobian
	error_f1.x=goal_f1.x-endeffector_f1.x;
	error_f2.x=goal_f2.x-endeffector_f2.x;
	error_f3.x=goal_f3.x-endeffector_f3.x;
	error_f4.x=goal_f4.x-endeffector_f4.x;

	error_f1.y=goal_f1.y-endeffector_f1.y;
	error_f2.y=goal_f2.y-endeffector_f2.y;
	error_f3.y=goal_f3.y-endeffector_f3.y;
	error_f4.y=goal_f4.y-endeffector_f4.y;

	C_eps_EG[1]=880*(error_f1.x+error_f2.x+error_f3.x+error_f4.x)/(4*fc[1]);
	C_eps_EG[0]=880*(error_f1.y+error_f2.y+error_f3.y+error_f4.y)/(4*fc[2]);

	//Pose estim
	x_kk[1][1]=goal_f1.x; x_kk[1][2]=goal_f2.x; x_kk[1][4]=goal_f4.x; x_kk[1][3]=goal_f3.x;
	x_kk[2][1]=goal_f1.y; x_kk[2][2]=goal_f2.y; x_kk[2][4]=goal_f4.y; x_kk[2][3]=goal_f3.y;

	estimPose();
	C_T_G[0][0]=Rckk[1][1]; C_T_G[0][1]=Rckk[1][2]; C_T_G[0][2]=Rckk[1][3]; C_T_G[0][3]=Tckk[1];
	C_T_G[1][0]=Rckk[2][1]; C_T_G[1][1]=Rckk[2][2]; C_T_G[1][2]=Rckk[2][3]; C_T_G[1][3]=Tckk[2];
	C_T_G[2][0]=Rckk[3][1]; C_T_G[2][1]=Rckk[3][2]; C_T_G[2][2]=Rckk[3][3]; C_T_G[2][3]=Tckk[3];
/*
C_T_G[0][0]=Rckk[1][1]; C_T_G[0][1]=Rckk[1][2]; C_T_G[0][2]=Rckk[1][3]; C_T_G[0][3]=Tckk[1];
C_T_G[1][0]=Rckk[2][1]; C_T_G[1][1]=Rckk[2][2]; C_T_G[1][2]=Rckk[2][3]; C_T_G[1][3]=Tckk[2];
C_T_G[2][0]=Rckk[3][1]; C_T_G[2][1]=Rckk[3][2]; C_T_G[2][2]=Rckk[3][3]; C_T_G[2][3]=Tckk[3];

	C_r_G[0]=C_T_G[3][0];
	C_r_G[1]=C_T_G[3][1];
	C_r_G[2]=C_T_G[3][2];
	C_r_G[5]=atan2(C_T_G[2][1],C_T_G[2][2]);
 	C_r_G[4]=atan2(-C_T_G[2][0],sqrt(C_T_G[0][0]*C_T_G[0][0]+C_T_G[1][0]*C_T_G[1][0]));
 	C_r_G[3]=atan2(C_T_G[1][0],C_T_G[0][0]);
*/

x_kk[1][1]=endeffector_f1.x; x_kk[1][2]=endeffector_f2.x; x_kk[1][4]=endeffector_f4.x; x_kk[1][3]=endeffector_f3.x;
x_kk[2][1]=endeffector_f1.y; x_kk[2][2]=endeffector_f2.y; x_kk[2][4]=endeffector_f4.y; x_kk[2][3]=endeffector_f3.y;



	estimPose();
//CAMERA
C_T_E[0][0]=Rckk[1][1]; C_T_E[0][1]=Rckk[1][2]; C_T_E[0][2]=Rckk[1][3]; C_T_E[0][3]=Tckk[1];
C_T_E[1][0]=Rckk[2][1]; C_T_E[1][1]=Rckk[2][2]; C_T_E[1][2]=Rckk[2][3]; C_T_E[1][3]=Tckk[2];
C_T_E[2][0]=Rckk[3][1]; C_T_E[2][1]=Rckk[3][2]; C_T_E[2][2]=Rckk[3][3]; C_T_E[2][3]=Tckk[3];

	C_r_G[0]=C_T_G[3][0];
	C_r_G[1]=C_T_G[3][1];
	C_r_G[2]=C_T_G[3][2];
	C_r_G[5]=atan2(C_T_G[2][1],C_T_G[2][2]);
 	C_r_G[4]=atan2(-C_T_G[2][0],sqrt(C_T_G[0][0]*C_T_G[0][0]+C_T_G[1][0]*C_T_G[1][0]));
 	C_r_G[3]=atan2(C_T_G[1][0],C_T_G[0][0]);


	C_r_E[0]=C_T_E[3][0];
	C_r_E[1]=C_T_E[3][1];
	C_r_E[2]=C_T_E[3][2];
	C_r_E[5]=atan2(C_T_E[2][1],C_T_E[2][2]);
 	C_r_E[4]=atan2(-C_T_E[2][0],sqrt(C_T_E[0][0]*C_T_E[0][0]+C_T_E[1][0]*C_T_E[1][0]));
 	C_r_E[3]=atan2(C_T_E[1][0],C_T_E[0][0]);

//EulerZYZ
//	C_r_E[5]=atan2(C_T_E[2][1],-C_T_E[2][0]);
// 	C_r_E[4]=atan2(sqrt(C_T_E[2][0]*C_T_E[2][0]+C_T_E[2][1]*C_T_E[2][1]),C_T_E[2][2]);
//	C_r_E[3]=atan2(C_T_E[1][2],C_T_E[0][2]);

/*
lib::Homog_matrix C_Tx_G;
lib::Homog_matrix C_Tx_E;
lib::Homog_matrix O_Tx_E;
lib::Homog_matrix O_Tx_C;
lib::Homog_matrix O_Tx_E__C;
lib::Homog_matrix O_Tx_G__C;

lib::Homog_matrix E_Tx_G;
lib::Homog_matrix E_Tx_G__O;
*/

for(int i=0; i<4; i++)
{
	for(int j=0; j<3; j++)
	{
		//C_Tx_G.matrix[i][j]=C_T_G[j][i];
		//C_Tx_E.matrix[i][j]=C_T_E[j][i];
		C_Tx_G.set_value(i,j,C_T_G[j][i]);
		C_Tx_E.set_value(i,j,C_T_E[j][i]);
	}
}



//O_Tx_C.set_xyz_euler_zyz(1.001, 0, 0.265, 0.0, 1.570, 3.140);
O_Tx_C.set_xyz_euler_zyz(0.0, 0, 0.0, 0.0, 1.570, 3.140);

//C_Tx_E.set_xyz_euler_zyz(10,10,1000, 0,0,0);

O_Tx_E__C=O_Tx_C*C_Tx_E;

double vec[6];
//double vec2[6];

C_Tx_G.get_xyz_euler_zyz(vec); //output is x,y,z, gamma,beta, alpha
//C_Tx_G.get_xyz_rpy(vec); //output is x,y,z, gamma,beta, alpha

//printf("GOAL POSE RPY %f %f %f %f %f %f\n", C_r_G[0], C_r_G[1], C_r_G[2], C_r_G[3], C_r_G[4], C_r_G[5]);
//printf("GOAL POSE HUB EUL %f %f %f %f %f %f\n", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);

O_Tx_E__C.get_xyz_euler_zyz(vec);
//C_Tx_E.get_xyz_rpy(vec); //output is x,y,z, gamma,beta, alpha

//printf("ENDEFFECTOR POSE RPY %f %f %f %f %f %f\n", C_r_E[0], C_r_E[1], C_r_E[2], C_r_E[3], C_r_E[4], C_r_E[5]);
//printf("ENDEFFECTOR POSE HUB EUL after rotation %f %f %f %f %f %f\n", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);

C_Tx_E.get_xyz_euler_zyz(vec);
//C_Tx_E.get_xyz_rpy(vec); //output is x,y,z, gamma,beta, alpha

//printf("ENDEFFECTOR POSE RPY %f %f %f %f %f %f\n", C_r_E[0], C_r_E[1], C_r_E[2], C_r_E[3], C_r_E[4], C_r_E[5]);
//printf("ENDEFFECTOR POSE HUB EUL %f %f %f %f %f %f\n", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);


//vec2[0]=1.001; vec2[1]=0; vec2[2]=0.265; vec2[3]=; vec2[4]=; vec2[5]=;

//O_Tx_E.set_xyz_euler_zyz(1.001, 0, 0.265, -0.0099, 1.570, 3.140);
O_Tx_E.set_xyz_euler_zyz(1.001, 0, 0.265, 0.0, 1.570, 3.140);
O_Tx_E.get_xyz_rpy(vec);

vec[3]=0;
//vec[5]=-atan2(C_Tx_E.matrix[1][0], C_Tx_E.matrix[1][1]);

//printf("\nENDEFFECTOR POSE RPY %f %f %f %f %f %f\n", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);

O_Tx_E__C.get_xyz_rpy(vec);

vec[3]=0;
//vec[5]=-atan2(O_Tx_E__C.matrix[1][0], O_Tx_E__C.matrix[1][1]);

//printf("\nENDEFFECTOR POSE FROM CAM RPY %f %f %f %f %f %f\n", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);

//cout << endeffector_f1.x<<"; " << endeffector_f2.x<<"; " << endeffector_f3.x<<"; " << endeffector_f4.x << endl;
//cout << endeffector_f1.y<<"; " << endeffector_f2.y<<"; " << endeffector_f3.y<<"; " << endeffector_f4.y << endl;

O_Tx_G__C=O_Tx_C*C_Tx_G;;

E_Tx_G=(!C_Tx_E)*C_Tx_G;
E_Tx_G__O=(!O_Tx_E__C)*O_Tx_G__C;

/*
cout<<"C_T_G "<<endl<<C_Tx_G<<endl;
cout<<"C_T_E "<<endl<<C_Tx_E<<endl;
cout<<"O_T_E__C "<<endl<<O_Tx_E__C<<endl;

cout<<"E_T_G "<<endl<<E_Tx_G<<endl;
cout<<"E_T_G__O "<<endl<<E_Tx_G__O<<endl;
*/

}

bool CMVision::estimPose4()
{
	objectClass();

	//0 - goal, 1 - endeffector
	estimPose2(0, goal_meanx, goal_meany, goal_matched_Z, goal_matched_pattern, goal_f1, goal_f2, goal_f3, goal_f4);
//	estimPose2(1, endeffector_meanx, endeffector_meany, endeffector_matched_Z, endeffector_matched_pattern, endeffector_f1, endeffector_f2, endeffector_f3, endeffector_f4);


	x_kk[1][1]=goal_f1.x; x_kk[1][2]=goal_f2.x; x_kk[1][4]=goal_f4.x; x_kk[1][3]=goal_f3.x;
	x_kk[2][1]=goal_f1.y; x_kk[2][2]=goal_f2.y; x_kk[2][4]=goal_f4.y; x_kk[2][3]=goal_f3.y;

	estimPose();

}

bool CMVision::estimError()
{
	objectClass();

	//0 - goal, 1 - endeffector
	estimPose2(0, goal_meanx, goal_meany, goal_matched_Z, goal_matched_pattern, goal_f1, goal_f2, goal_f3, goal_f4);
	estimPose2(1, endeffector_meanx, endeffector_meany, endeffector_matched_Z, endeffector_matched_pattern, endeffector_f1, endeffector_f2, endeffector_f3, endeffector_f4);

	error_f1.x=goal_f1.x-endeffector_f1.x;
	error_f2.x=goal_f2.x-endeffector_f2.x;
	error_f3.x=goal_f3.x-endeffector_f3.x;
	error_f4.x=goal_f4.x-endeffector_f4.x;

	error_f1.y=goal_f1.y-endeffector_f1.y;
	error_f2.y=goal_f2.y-endeffector_f2.y;
	error_f3.y=goal_f3.y-endeffector_f3.y;
	error_f4.y=goal_f4.y-endeffector_f4.y;

	C_eps_EG[1]=880*(error_f1.x+error_f2.x+error_f3.x+error_f4.x)/(4*fc[1]);
	C_eps_EG[0]=880*(error_f1.y+error_f2.y+error_f3.y+error_f4.y)/(4*fc[2]);

	//cout << "XY: " << C_eps_EG[0] <<"  "<< C_eps_EG[1] << endl;
}
