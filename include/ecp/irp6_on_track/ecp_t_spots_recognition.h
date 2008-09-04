/*!
 * \file ecp_t_spotsrecognition.h
 * \brief Declaration of a class responsible
 * for robot moving, used by Piotr Sakowicz.
 * - class declaration
 * \author vented.baffle
 * \date 21.08.2008
 */

#ifndef _ECP_T_SpotsRecognition_H_
#define _ECP_T_SpotsRecognition_H_

#include "ecp/common/ecp_task.h"
#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

//fradia
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_spots_recognition.h"

#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_sleep.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>


class ecp_t_spots_recognition: public ecp_task
{
		/*!
		 * katalog, w ktorym sa trajektorie
		 */
		char * katalog_traj;
		/*!
		 * katalog, w ktorym beda zapisywane dane kalibracyjne
		 */
		char * katalog_dump;

	protected:

		ecp_spots_generator* generator;
		ecp_smooth_generator* smooth;
		ecp_sleep_generator* sleep_g;

		/*!
        * Generator used for communication with cvFraDIA.
        */
	    //ecp_cvfradia_generator* cvg;

	public:
		ecp_t_spots_recognition(configurator &_config);
		~ecp_t_spots_recognition();

		void task_initialization(void);
		void main_task_algorithm(void);
};

#endif

