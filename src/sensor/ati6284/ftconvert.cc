/*	! \file src/sensor/ati6284/ftconvert.cc
    * \brief ATIDAQ F/T C Library v1.0.2
     * Copyright (c) 2001 ATI Industrial Automation
     *
     * The MIT License
     * 
     * Permission is hereby granted, free of charge, to any person obtaining a
     * copy of this software and associated documentation files (the "Software"),
     * to deal in the Software without restriction, including without limitation
     * the rights to use, copy, modify, merge, publish, distribute, sublicense,
     * and/or sell copies of the Software, and to permit persons to whom the
     * Software is furnished to do so, subject to the following conditions:
     * 
     * The above copyright notice and this permission notice shall be included
     * in all copies or substantial portions of the Software.
     * 
     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
     * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
     * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
     * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
     * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
     * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
     * OTHER DEALINGS IN THE SOFTWARE.
     */
 
#include <cstdio>
#include <cstdlib>
#include "ftconfig.h"
#include "display.h"

extern short int invalid_value;	
extern Calibration *cal;				//!<  struct containing calibration information
extern char *calfilepath;		//!<  sciezka do pliku z kalibracja
extern unsigned short index; 						//!<  index po ktorym czyta w pliku konfiguracyjnym (domysnie 1)

int ftconvert(float SampleReading[6],float SampleBias[6],float FT[6]) {
	unsigned short i;       		//!<  loop variable used to print results
	short sts;              			//!<  return value from functions
	
	//!<  This sample transform includes a translation along the Z-axis and a rotation about the X-axis.
	float SampleTT[6]={0,0,0,0,0,0};
	
#if INFO
	DisplayInfo(cal,calfilepath,index);	//!<  wyswietlenie informacji o czujniku
#endif

	// Set force units.
	// This step is optional; by default, the units are inherited from the calibration file.
	/*sts=SetForceUnits(cal,(char*)"N");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid force units"); return 0;
		default: printf("Unknown error"); return 0;
	}

	// Set torque units.
	// This step is optional; by default, the units are inherited from the calibration file.
	sts=SetTorqueUnits(cal,(char*)"N-m");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid torque units"); return 0;
		default: printf("Unknown error"); return 0;
	}*/

	// Set tool transform.
	// This line is only required if you want to move or rotate the sensor's coordinate system.
	// This example tool transform translates the coordinate system 20 mm along the Z-axis 
	// and rotates it 45 degrees about the X-axis.
	sts=SetToolTransform(cal,SampleTT,(char*)"mm",(char*)"degrees");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid distance units"); return 0;
		case 3: printf("Invalid angle units"); return 0;
		default: printf("Unknown error"); return 0;
	}

	// Temperature compensation is on by default if it is available.
	// To explicitly disable temperature compensation, uncomment the following code
	// SetTempComp(cal,false);                   // disable temperature compensation
	// switch (sts) {
	// 	case 0: break;	// successful completion
	// 	case 1: printf("Invalid Calibration struct"); return 0;
	// 	case 2: printf("Temperature Compensation not available on this transducer"); return 0;
	// 	default: printf("Unknown error"); return 0;
	// }

	//!<  store an unloaded measurement; this removes the effect of tooling weight
	Bias(cal,SampleBias);

	//!<  convert a loaded measurement into forces and torques
	ConvertToFT(cal,SampleReading,FT);		

	for(i=0;i<6;i++)	{
		if(((cal->MaxLoads[i])<FT[i])||((cal->MaxLoads[i])<-FT[i]))
			invalid_value=1;
	}

#if WYNIKI
	printf("\n-----------------------------------------------------------------------------------------------\n");
	printf("%c\t%c\t%c\t%c\t%c\t%c\n",'X','Y','Z','X','Y','Z');
	printf("\n-----------------------------------------------------------------------------------------------\n");
	printf("Bias reading:\n");
	for (i=0;i<6;i++)
		printf("%9.6f[V] ",SampleBias[i]);		
	printf("\n\nMeasurement:\n\n");
	for (i=0;i<6;i++)
		printf("%9.6f[V] ",SampleReading[i]);		
	printf("\n\nDifference Measurement - Bias reading :\n\n");
	for (i=0;i<6;i++)
		printf("%9.6f[V] ",SampleReading[i]-SampleBias[i]);
	printf("\n\nResult (force/torque):\n\n");
	for (i=0;i<6;i++){
		if (i<3)
			printf("%9.6f[N] ",FT[i]);
		else
			printf("%9.6f[N-m] ",FT[i]);		
	}
	printf("\n-----------------------------------------------------------------------------------------------\n");
	
#endif

	return 0;
}


int DisplayInfo(Calibration *cal,const char *calfilepath,unsigned short index){
	int i,j; //!<  liczniki
	//!<  display info from calibration file
	printf("Calibration Information for %s, index #%i\n",calfilepath,index);
	printf("                  Serial: %s\n",cal->Serial);
	printf("              Body Style: %s\n",cal->BodyStyle);
	printf("             Calibration: %s\n",cal->PartNumber);
	printf("        Calibration Date: %s\n",cal->CalDate);
	printf("                  Family: %s\n",cal->Family);
	printf("              # Channels: %i\n",cal->rt.NumChannels);
	printf("                  # Axes: %i\n",cal->rt.NumAxes);
	printf("             Force Units: %s\n",cal->ForceUnits);
	printf("            Torque Units: %s\n",cal->TorqueUnits);
	printf("Temperature Compensation: %s\n",(cal->TempCompAvailable ? "Yes" : "No"));
	
	//!<  print maximum loads of axes
	printf("\nRated Loads\n");
	for (i=0;i<cal->rt.NumAxes;i++) {
		char *units;
		if ((cal->AxisNames[i])[0]=='F') {
			units=cal->ForceUnits;
		} else units=cal->TorqueUnits;
		printf("%s: %f %s\n",cal->AxisNames[i],cal->MaxLoads[i],units);
	}

	//!<  print working calibration matrix
	printf("\nWorking Calibration Matrix\n");
	printf("     ");
	for (i=0;i<cal->rt.NumChannels-1;i++)
		printf("G%i            ",i);
	printf("\n");
	for (i=0;i<cal->rt.NumAxes;i++) {
		printf("%s: ",cal->AxisNames[i]);
		for (j=0;j<cal->rt.NumChannels-1;j++)
			printf("%13.5e ",cal->rt.working_matrix[i][j]);
		printf("\n");
	}

	//!<  print temperature compensation information, if available
	if (cal->TempCompAvailable) {
		printf("\nTemperature Compensation Information\n");
		printf("BS: ");
		for (i=0;i<cal->rt.NumChannels-1;i++) {
			printf("%13.5e ",cal->rt.bias_slopes[i]);
		}
		printf("\nGS: ");
		for (i=0;i<cal->rt.NumChannels-1;i++) {
			printf("%13.5e ",cal->rt.gain_slopes[i]);
		}
		printf("\nTherm: %f\n",cal->rt.thermistor);
	}		
	return 1;
}

