 // -------------------------------------------------------------------------
//                             vsp_s.cc 		dla QNX6.2
//
//             Virtual Sensor Process (VSP) - Speach recognizin'
//
// last update: 14.11.04
// Author: mstaniak@elka.pw.edu.pl [based on tkornuta's vsp templates]
// 
// -------------------------------------------------------------------------

#include <stdio.h>
#include <sys/neutrino.h>
#include <string.h>
#include <sys/select.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_mic.h"

// Konfigurator
#include "lib/configurator.h"

//SOUND
//#include <fcntl.h> 
#include <gulliver.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/termio.h>

#include <sys/asoundlib.h>
#include "sound.h"

#include "rosm.cc"

CROSMDoc r;

/*
#include "typedefs.h"
#include "impconst.h"
#include "com_buf.h"
#include "lst.h"
#include "srplib.h"
#include "vsp_s.h"
*/

namespace mrrocpp {
namespace vsp {
namespace sensor {


#define TRUE 1
#define FALSE 0


// extern pid_t UI_pid;           // identyfikator procesu UI
// extern lib::configurator* config;

base* return_created_sensor (lib::configurator &_config)
{
	return new mic(_config);
}// : return_created_sensor


// Rejstracja procesu VSP
mic::mic(lib::configurator &_config) : base(_config){

//	unsigned long int e;			// kod bledu systemowego	 
	printf("Konstruktor VSP! - SOUND\n");
	is_sensor_configured=FALSE;	// czujnik niezainicjowany 
	is_reading_ready=FALSE;				// nie ma zadnego gotowego odczytu
	
	ThreadCtl (_NTO_TCTL_IO, NULL);  // by YOYEK & 7 - nadanie odpowiednich uprawinien watkowi 	
	
	//i = 0;
	//state = 0;

	card = -1;
    dev  = 0;

    mSampleRate = 16000;
    mSampleChannels = 1; //2
    mSampleBits = 16;
    mSampleTime = 5; //3
    
   
   
    
    //preparin' listenin'
    setvbuf (stdin, NULL, _IONBF, 0);
    if (card == -1)
    {
        if ((rtn = snd_pcm_open_preferred (&pcm_handle, &card, &dev, SND_PCM_OPEN_CAPTURE)) < 0)
            ;
            //return err ("device open");
    }
    else
    {
        if ((rtn = snd_pcm_open (&pcm_handle, card, dev, SND_PCM_OPEN_CAPTURE)) < 0)
            ;
            //return err ("device open");
    }
	
    mSamples = mSampleRate * mSampleChannels * mSampleBits / 8 * mSampleTime;
    

    *(short *) riff_hdr.wave.fmt.voices = ENDIAN_LE16 (mSampleChannels);
    *(long *) riff_hdr.wave.fmt.rate = ENDIAN_LE32 (mSampleRate);
    *(long *) riff_hdr.wave.fmt.char_per_sec =
        ENDIAN_LE32 (mSampleRate * mSampleChannels * mSampleBits / 8);
    *(short *) riff_hdr.wave.fmt.block_align = ENDIAN_LE16 (mSampleChannels * mSampleBits / 8);
    *(short *) riff_hdr.wave.fmt.bits_per_sample = ENDIAN_LE16 (mSampleBits);
    *(long *) riff_hdr.wave.data.data_len = ENDIAN_LE32 (mSamples);
    *(long *) riff_hdr.wave_len = ENDIAN_LE32 (mSamples + sizeof (riff_hdr) - 8);
    
    file1 = fopen ("from_vsp.wav", "w");
    fwrite (&riff_hdr, 1, sizeof (riff_hdr), file1);

mSamples = mSampleRate * mSampleChannels  * mSampleTime;

    //printf ("SampleRate = %d, Channels = %d, SampleBits = %d\n", mSampleRate, mSampleChannels,
    //    mSampleBits);

    /* disabling mmap is not actually required in this example but it is included to 
     * demonstrate how it is used when it is required.
     */
    if ((rtn = snd_pcm_plugin_set_disable (pcm_handle, PLUGIN_DISABLE_MMAP)) < 0)
    {
        fprintf (stderr, "snd_pcm_plugin_set_disable failed: %s\n", snd_strerror (rtn));
        ;//return -1;        
    }

    memset (&pi, 0, sizeof (pi));
    pi.channel = SND_PCM_CHANNEL_CAPTURE;
    if ((rtn = snd_pcm_plugin_info (pcm_handle, &pi)) < 0)
    {
        fprintf (stderr, "snd_pcm_plugin_info failed: %s\n", snd_strerror (rtn));
        ;//return -1;
    }

    memset (&pp, 0, sizeof (pp));

    pp.mode = SND_PCM_MODE_BLOCK;
    pp.channel = SND_PCM_CHANNEL_CAPTURE;
    pp.start_mode = SND_PCM_START_DATA;
    pp.stop_mode = SND_PCM_STOP_STOP;

    pp.buf.block.frag_size =1;// pi.max_fragment_size;
    pp.buf.block.frags_max = -1;
    pp.buf.block.frags_min = 1;

    pp.format.interleave = 1;
    pp.format.rate = mSampleRate;
    pp.format.voices = mSampleChannels;

    if (mSampleBits == 8)
        pp.format.format = SND_PCM_SFMT_U8;
    else
        pp.format.format = SND_PCM_SFMT_S16_LE;

    if ((rtn = snd_pcm_plugin_params (pcm_handle, &pp)) < 0)
    {
        fprintf (stderr, "snd_pcm_plugin_params failed: %s\n", snd_strerror (rtn));
        ;//return -1;
    }

    if ((rtn = snd_pcm_plugin_prepare (pcm_handle, SND_PCM_CHANNEL_CAPTURE)) < 0)
        fprintf (stderr, "snd_pcm_plugin_prepare failed: %s\n", snd_strerror (rtn));


    memset (&setup, 0, sizeof (setup));
    memset (&group, 0, sizeof (group));
    setup.channel = SND_PCM_CHANNEL_CAPTURE;
    setup.mixer_gid = &group.gid;
    if ((rtn = snd_pcm_plugin_setup (pcm_handle, &setup)) < 0)
    {
        fprintf (stderr, "snd_pcm_plugin_setup failed: %s\n", snd_strerror (rtn));
        ;//return -1;
    }
    printf ("Format %s \n", snd_pcm_get_format_name (setup.format.format));
    printf ("Frag Size %d \n", setup.buf.block.frag_size);
    printf ("Rate %d \n", setup.format.rate);
    bsize = 1024;// setup.buf.block.frag_size;

    if (group.gid.name[0] == 0)
    {
        printf ("Mixer Pcm Group [%s] Not Set \n", group.gid.name);
        printf ("***>>>> Input Gain Controls Disabled <<<<*** \n");
    }
    else
        printf ("Mixer Pcm Group [%s]\n", group.gid.name);
    if ((rtn = snd_mixer_open (&mixer_handle, card, setup.mixer_device)) < 0)
    {
        fprintf (stderr, "snd_mixer_open failed: %s\n", snd_strerror (rtn));
        ;//return -1;
    }

    mSampleBfr1 = new short[mSamples]; //malloc (bsize); //char _. short
    FD_ZERO (&rfds);
    FD_ZERO (&wfds);
	
	//CROSMDoc r;
	//r.LA_KLAS=4;
	std::cout << "ROSM" << std::endl;
	r.WczytajKlasy();
};

mic::~mic(void){
	
	rtn = snd_mixer_close (mixer_handle);
    rtn = snd_pcm_close (pcm_handle);
    fclose (file1);
	printf("Destruktor VSP\n");

};

/**************************** inicjacja czujnika ****************************/
void mic::configure_sensor (void){

	is_sensor_configured=TRUE;
  	 sr_msg->message ("Sensor MIC initiated"); // 7 
};

/*
void vsp_mic::wait_for_event(){

	if(interatt==0){
	memset(&event, 0, sizeof(event));//by y&w
	event.sigev_notify = SIGEV_INTR;//by y&w
	if ( (id =InterruptAttach (irq_no, int_handler, (void *) &md , sizeof(md), 0)) == -1)
		  printf( "Unable to attach interrupt handler: \n");
	interatt=1;
	};
	InterruptWait (NULL, NULL);

};	
*/ // useless in no-wait interactive mode 

/*************************** inicjacja odczytu ******************************/
void mic::initiate_reading (void){
  
//	if(!is_sensor_configured)
//	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED); //MAC7 odrem w MRROC++
 	
	//listenin' to sounds
	n = 1;
	N=0; 
    //state=0;
    std::cout << "INIT READ" << std::endl;
    
   // while (N < mSamples && n > 0)
   // {
        FD_SET (STDIN_FILENO, &rfds);
        FD_SET (snd_mixer_file_descriptor (mixer_handle), &rfds);
        FD_SET (snd_pcm_file_descriptor (pcm_handle, SND_PCM_CHANNEL_CAPTURE), &rfds);


        if (FD_ISSET (snd_pcm_file_descriptor (pcm_handle, SND_PCM_CHANNEL_CAPTURE), &rfds))
        {
            snd_pcm_channel_status_t status;
            int     read = 0;

            //read = snd_pcm_plugin_read (pcm_handle, mSampleBfr1, bsize);
			read = snd_pcm_plugin_read (pcm_handle, mSampleBfr1, mSamples);
		
			delete [] r.probki; // WK : trzeba usunac dotychczasowa pamiec probek

			long tmp_liczba_probek;
			tmp_liczba_probek= mSamples; // Rzeczywista liczba probek //MAC7
		
			if (tmp_liczba_probek <  2 * r.kolumnyUproszcz * r.okno)
				r.liczba_probek = 2 * r.kolumnyUproszcz * r.okno;
			else r.liczba_probek = tmp_liczba_probek;


			r.probki= new double[r.liczba_probek];
			if (r.liczba_probek > tmp_liczba_probek)
			{
				for (i= tmp_liczba_probek; i<r.liczba_probek; i++) r.probki[i]= 0.0;
			}
			if (mSampleBits == 16)   //gdy probki 16 bitowe to przepisz tablice //MAC7
			{	
				//short *probki_short= (short*)tempWave.GetBuffer();
				for (i=0; i<tmp_liczba_probek; i++) r.probki[i] = mSampleBfr1[i]; //probki_short[i]; 
				//for (i=0; i<10; i++) std::cout << mSampleBfr1[i] << "->" << r.probki[i] << ",";
			}
			else if (mSampleBits == 32) //MAC7
			{
				/* WK begin */
				//int *probki_long= (int *)tempWave.GetBuffer();
		 		for (i=0; i< tmp_liczba_probek; i++) r.probki[i]= mSampleBfr1[i]; // probki_long[i]; 
				/* WK end */
		 		// probki= (double *)tempWave.GetBuffer(); //wpw ustaw probki 32 bitowe
			}
			else
			{
				std::cout << "Bad fromat" << std::endl; 
		 		//return 0;
			}
			std::cout << std::endl;
    
    		double tmp;
			long i;
			r.maxWartosc = fabs(r.probki[1]); // Potzebna jedynie dla wstpenej wizualizacji
			for(i=1; i < r.liczba_probek; i++)
			{
				tmp = fabs(r.probki[i]);
				if (tmp > r.maxWartosc) r.maxWartosc = tmp; 
			}
			
			r.InicjujAnalize(1); // -- bez przerw na wizualizacj�. 	
			r.bezSzumu = true;
			r.poczatek_ramki = r.ZnajdzRamke(1); // Zakladamy najpierw szeroki spektrogram 			
			r.ObliczCechyOkien(1, 0); // Nie przerywaj pracy wizualizacj�
			r.RozpoznajKomende();

		//primitive speach recog. - checking: is sth speakin'?        
		/*
      	for(i=0; i<bsize; i++)
        {
            		
            		if(mSampleBfr1[i]>120 && state==0) //-120 MAC7
            		{ 
            			state=N*bsize+i; 
            			//printf("OK - >%d  %d\n",N*bsize+i, mSampleBfr1[i]);
            			//srp_msg->message ("OK"); 
						          
            		}
            		
            		if(mSampleBfr1[i]<=120 && state!=0 && (N*bsize+i-state)>20000000)
            		{ 
            			state=0;
            			//printf("KONIEC - >%d  %d\n",N*bsize+i, mSampleBfr1[i]);
           			}
           }
            //fwrite (mSampleBfr1, 1, read, file1);
             //fwrite (mSampleBfr1, 1, read, file2);
            N += read;
  */
        }
//    } 
 printf("END LISTENIN' %d\n",r.poczatek_ramki);
  fwrite (mSampleBfr1, 1,  mSamples, file1);
 
      printf("END LISTENIN'2\n");
	 // end of listening
		
	
	is_reading_ready=TRUE;							// odczyt jakikolwiek	
   
}; //wait_for_event
		
/***************************** odczyt z czujnika *****************************/
void mic::get_reading (void){

	//if(!is_sensor_configured)
	//     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED); //MAC7 odrem w MRROC++
	
	// jezeli chcemy jakikolwiek odczyt	-> is_reading_ready
	//if(!is_reading_ready)
	//     throw sensor_error (FATAL_ERROR, READING_NOT_READY);    //MAC7 odrem w MRROC++

	//from_vsp.vsp_report=VSP_REPLY_OK; //MAC7 odrem w MRROC++
	
	// tutaj: czujnik skalibrowany, odczyt dokonany, zapisany w "image", przepisanie wszystkich pol
	// przepisanie do bufora komunikacyjnego
	
	//from_vsp.comm_image.ear.is_speaking = 0; //MAC7 odrem w MRROC++
	//from_vsp.comm_image.ear.text = "OK"; //MAC7 odrem w MRROC++
	
	
	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// jezeli chcemy jakikolwiek odczyt	-> is_reading_ready
	// printf("7 - still reading %d\n",is_reading_ready);
	if(!is_reading_ready)
	     throw sensor_error (FATAL_ERROR, READING_NOT_READY);   

	// ok
	from_vsp.vsp_report=VSP_REPLY_OK;

	
	// fill up frame
/*
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			from_vsp.comm_image.sensor_union.camera.frame[4*i+j]=r.recog_word_id;
	for(int i=0; i<3; i++)
			from_vsp.comm_image.sensor_union.camera.frame[4*i+3]=7; //vision.cube_center[i+1];
	for(int j=0; j<3; j++)
			from_vsp.comm_image.sensor_union.camera.frame[12+j]=0;

			from_vsp.comm_image.sensor_union.camera.frame[15]=1;
*/	
	from_vsp.comm_image.sensor_union.mic.word_id=r.recog_word_id;
	
	// for(int i=0; i<16; i++)
	// 	from_vsp.comm_image.sensor_union.camera.frame[i] = 0.5;
     // sr_msg->message ("VSP Get reading ok");   
     is_reading_ready=false; // 7
	
    // is_reading_ready=FALSE;
	};
} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

