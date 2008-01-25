// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			ecp_mp_s_mic.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#ifndef __ECP_MP_S_VIS_SAC_LX_H
#define __ECP_MP_S_VIS_SAC_LX_H

#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa sensor

/***************** Klasa czujnikow ********************/
class ecp_mp_vis_sac_lx_sensor: public ecp_mp_sensor{
  private:									// pola do komunikacji

  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	ecp_mp_vis_sac_lx_sensor (SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object);
											// konstruktor czujnika virtualnego
	void get_reading (void);			// odebranie odczytu od VSP
}; 

#endif
