/*
 * EcpGAndroid.h
 *
 *  Created on: Nov 20, 2011
 *      Author: hh7
 */

#ifndef ECPGANDROID_H_
#define ECPGANDROID_H_




#include "base/lib/mrmath/mrmath.h"

#include "base/ecp/ecp_generator.h"
#include "application/android_teach/sensor/EcpMpAndroid.h"
#include "application/android_teach/AndroidState.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

#define MAX_NO_OF_DEGREES 8

#define NO_OF_DEGREES 8

#if (MAX_NO_OF_DEGREES < NO_OF_DEGREES)
#error MAX_NO_OF_DEGREES exceeded
#endif



class EcpGAndroid : public common::generator::generator
{

private:
    //ilosc krokow
	int max_steps;

	int step_no;
    //kontroler


protected:
	ecp_mp::sensor::EcpMpAndroid* androidSensor;
	ecp_mp::sensor::android_teach::AndroidState* androidState;


    double currentValue[MAX_NO_OF_DEGREES];
    double currentGripperValue;
    double requestedChange[MAX_NO_OF_DEGREES];
    double nextChange[MAX_NO_OF_DEGREES];
    double maxChange[MAX_NO_OF_DEGREES];
    double multipliers[MAX_NO_OF_DEGREES];


  	//numer kroku
    bool releasedA;
    bool limit;
    bool stop;

    lib::Homog_matrix homog_matrix;

public:

/**
 * Tworzy generator odtwarzajacy orientacje kontrolera
 * @param zadanie
 * @param major_axis wartosc wiekszej polosi
 * @param minor_axis wartosc mniejszej polosi
 * @author jedrzej
 */
	EcpGAndroid(common::task::task& _ecp_task,ecp_mp::sensor::EcpMpAndroid* androidSensor, ecp_mp::sensor::android_teach::AndroidState* androidState);
	virtual ~EcpGAndroid();
    /**
     * Generuje pierwszy krok
     * @author jedrzej
     */
    virtual bool first_step() = 0;

    /**
     * Generuje kolejne punkty wynikajace z aktualnej orientacji kontrolera
     * @author jedrzej
     */
    virtual bool next_step();

    void execute_motion(void);

    virtual void preset_position(void) = 0;

    virtual void set_position(bool changed) = 0;

    bool calculate_change(int axis, double value);

    int get_axis(void);

};

/** @} */ // end of wii_teach

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
#endif /* ECPGANDROID_H_ */
