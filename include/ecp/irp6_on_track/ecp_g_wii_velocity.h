#if !defined_ecp_wii_velocity_generator_H
# define _ecp_wii_velocity_generator_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_g_force.h"

class ecp_wii_velocity_generator : public ecp_tff_nose_run_generator
{
	public:
	/**
	 * Tworzy generator odtwarzajacy orientacje kontrolera
	 * @param zadanie
	 * @param major_axis wartosc wiekszej polosi
	 * @param minor_axis wartosc mniejszej polosi
	 * @author jedrzej
	 */
    ecp_wii_velocity_generator (ecp_task& _ecp_task);

    /**
     * Generuje pierwszy krok
     * @author jedrzej
     */
    virtual bool first_step();

    /**
     * Generuje kolejne punkty wynikajace z aktualnej orientacji kontrolera
     * @author jedrzej
     */
    virtual bool next_step();

    void execute_motion(void);
};

#endif
