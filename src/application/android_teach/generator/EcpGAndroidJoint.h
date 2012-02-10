/*
 * EcpGAndroidJoint.h
 *
 *  Created on: Nov 20, 2011
 *      Author: hh7
 */

#ifndef ECPGANDROIDJOINT_H_
#define ECPGANDROIDJOINT_H_


#include "base/ecp/ecp_generator.h"
#include "application/android_teach/sensor/EcpMpAndroid.h"
#include "application/android_teach/generator/EcpGAndroid.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

class EcpGAndroidJoint : public generator::EcpGAndroid
{

public:
/**
 * Tworzy generator odtwarzajacy orientacje kontrolera
 * @param zadanie
 * @param major_axis wartosc wiekszej polosi
 * @param minor_axis wartosc mniejszej polosi
 * @author jedrzej
 */

	EcpGAndroidJoint(common::task::task& _ecp_task,ecp_mp::sensor::EcpMpAndroid* androidSensor, ecp_mp::sensor::android_teach::AndroidState* androidState);
	virtual ~EcpGAndroidJoint();
    /**
     * Generuje pierwszy krok
     * @author jedrzej
     */
    virtual bool first_step();

    void preset_position(void);

    virtual void set_position(bool changed);
};

/** @} */ // end of wii_teach

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
#endif /* ECPGANDROIDJOINT_H_ */
