#include "vsp/vis/cube.h"
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <stdio.h>
#include <string.h>

inline unsigned int max(unsigned int a,unsigned int b)
{
  return((a > b)? a : b);
}

// returns minimal value of two parameters

inline unsigned int min(unsigned int a,unsigned int b)
{
  return((a < b)? a : b);
}

void RubiksCube::clear()
{

	tiles_count=0;
	tiles_area.min=0;
	tiles_area.max=0;
	tiles_area.average=0;
	roi.x1=1000;
	roi.x2=0;
	roi.y1=1000;
	roi.y2=0;
  	for(int i=0;i<RUBIK_MAX_TILES;i++)
	{
		tiles[i].x1=0;
		tiles[i].x2=0;
		tiles[i].y1=0;
		tiles[i].y2=0;
		tiles[i].area=0;
		tiles[i].color=0;
		for(int j=0;j<4;j++)
		{
			tiles[i].vertices[j].x=0;
			tiles[i].vertices[j].y=0;
		}
		tiles[i].center_v.x=0;
		tiles[i].center_v.y=0;
		tiles[i].center.x=0;
		tiles[i].center.y=0;
	}

}

void RubiksCube::close()
{

}

rubik_compare RubiksCube::compare(RubiksCube *k)
{
	rubik_compare compare;
	compare.tiles_new_count=0;
	compare.tiles_old_count=0;

	compare.tiles_count_diff=tiles_count-k->tiles_count; //roznica w liczbie kafelkow


	// sprawdzenie czy odpowiadajace kafelki sa podobne
	point_d shift=findShift(k);
//	int found;
	for(int i=0;i<tiles_count;i++)
	{
		compare.tiles[i]=findTile(i,k);

		if(compare.tiles[i]!=-1)
		{
			compare.tiles_old_count++;

		}
		else
			compare.tiles_new_count++;
	}

	return(compare);

}


int RubiksCube::findTile(int nr,RubiksCube *k)
{
	/* szuka w poblizu spodziewanej pozycji kafelka o zblizonych parametrach */
	//point_d shift=findShift(k);
	double prec1=0.15, prec2=0.20;
	int x1,x2,y1,y2,a1,a2,c1,c2;
	int i,j,tmp_count;
	// point_d t1=tiles[nr].center, t2;
	int suspected[10]; //tu znajda sie numery kafelkow podejrzanych
	int min_dist,tmp;
	int suspected_count=0;
	//znalezienie wszystkich kafelkow o tym samym kolorze
	for(i=0;i<k->tiles_count;i++)
		if(k->tiles[i].color==tiles[nr].color)	suspected[suspected_count++]=i;

	//wsrod podejrzanych szukamy zgodnych co do powierzchni
	tmp_count=suspected_count;
	suspected_count=0;
	for(i=0;i<tmp_count;i++)
	{
		j=suspected[i]; //nr podejrzewanego kafelka
		if(k->tiles[j].area>tiles[nr].area*(1-prec1)&&k->tiles[j].area<tiles[nr].area*(1+prec1))
			suspected[suspected_count++]=j;
	}
	//a nastepnie co do obwodu
	tmp_count=suspected_count;
	suspected_count=0;
	for(i=0;i<tmp_count;i++)
	{
		j=suspected[i]; //nr podejrzewanego kafelka
		if(k->tiles[j].contour_length>tiles[nr].contour_length*(1-prec2)&&k->tiles[j].contour_length<tiles[nr].contour_length*(1+prec2))
			suspected[suspected_count++]=j;
	}
	//na koncu szukamy najblizszego jesli jest wiecej podejrzanych
	//ale najblizszego nie tylko w sensie odleglosci, ale w sensie wszystkich parametrow
	if(suspected_count>1)
	{
		tmp_count=suspected_count;
		suspected_count=1; //na pewno pozostanie jeden kandydat!
		min_dist=0xffffffff;
		for(i=0;i<tmp_count;i++)
		{

			x1=(int)tiles[nr].center.x;
			y1=(int)tiles[nr].center.y;
			a1=tiles[nr].area;
			c1=tiles[nr].contour_length;
			j=suspected[i]; //nr podejrzewanego kafelka
			//wyznaczamy te same wartosci dla kandydata
			x2=(int)k->tiles[j].center.x;
			y2=(int)k->tiles[j].center.y;
			a2=k->tiles[j].area;
			c2=k->tiles[j].contour_length;

			//normalizujemy wszystkie parametry zeby skala nie wplywala na wynik
			tmp=max(x1,y1); tmp=max(tmp,x2); tmp=max(tmp,y2);
			x1=x1*1000/tmp; y1=y1*1000/tmp; x2=x2*1000/tmp; y2=y2*1000/tmp;
			tmp=max(a1,a2);
			a1=a1*1000/tmp; a2=a2*1000/tmp;
			tmp=max(c1,c2);
			c1=c1*1000/tmp; c2=c2*1000/tmp;

			tmp=(x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(a1-a2)*(a1-a2)+(c1-c2)*(c1-c2);

			if(tmp<min_dist)
			{
				suspected[0]=j;
				min_dist=tmp;
			}

		}
	}
if(suspected_count==1)
	return(suspected[0]);
else
	return(-1);
}

bool RubiksCube::build(CMVision *vision)
{
	int i,j;

	tiles_count=vision->filtered_blobs_count;
	if(tiles_count>RUBIK_MAX_TILES || tiles_count==0) return false;

	tiles_area.min=500000;
	tiles_area.max=0;
	tiles_area.average=0;
	tiles_contour.min=500000;
	tiles_contour.max=0;
	tiles_contour.average=0;


	for(i=0;i<tiles_count;i++)
	{
		tiles[i].x1=vision->blobs[vision->filtered_blobs[i]].x1;
		tiles[i].x2=vision->blobs[vision->filtered_blobs[i]].x2;
		tiles[i].y1=vision->blobs[vision->filtered_blobs[i]].y1;
		tiles[i].y2=vision->blobs[vision->filtered_blobs[i]].y2;
		tiles[i].area=vision->blobs[vision->filtered_blobs[i]].area;
		tiles[i].contour_length=vision->blobs[vision->filtered_blobs[i]].contour_length;
		tiles[i].color=vision->blobs[vision->filtered_blobs[i]].color;
		tiles_area.min=min(tiles_area.min, tiles[i].area);
		tiles_area.max=max(tiles_area.max, tiles[i].area);
		tiles_area.average+=tiles[i].area;
		tiles_contour.min=min(tiles_contour.min, tiles[i].contour_length);
		tiles_contour.max=max(tiles_contour.max, tiles[i].contour_length);
		tiles_contour.average+=tiles[i].contour_length;
		for(j=0;j<4;j++)
		{
			tiles[i].vertices[j]=vision->blobs[vision->filtered_blobs[i]].vertices[j];
			tiles[i].center_v.x+=tiles[i].vertices[j].x;
			tiles[i].center_v.y+=tiles[i].vertices[j].y;
		}
		tiles[i].center_v.x/=4;
		tiles[i].center_v.y/=4;
		tiles[i].center=vision->blobs[vision->filtered_blobs[i]].center;

		if(tiles[i].x1<roi.x1) roi.x1=tiles[i].x1;
		if(tiles[i].x2>roi.x2) roi.x2=tiles[i].x2;
		if(tiles[i].y1<roi.y1) roi.y1=tiles[i].y1;
		if(tiles[i].y2>roi.y2) roi.y2=tiles[i].y2;

	}
	tiles_area.average/=tiles_count;
	tiles_contour.average/=tiles_count;
	roi.center.x=(roi.x2+roi.x1)/2.0;
	roi.center.y=(roi.y2+roi.y1)/2.0;
	return true;
}



point_d RubiksCube::findShift(RubiksCube *k){
	point_d shift;
	shift.x=k->roi.center.x-roi.center.x;
	shift.y=k->roi.center.y-roi.center.y;

	return(shift);
}
