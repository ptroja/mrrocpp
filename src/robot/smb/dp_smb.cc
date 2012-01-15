/*!
 * @file
 * @brief File contains dp_smb class definition for SwarmItFix smb
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include <cmath>
#include <cstring>
#include <ostream>

#include "dp_smb.h"

namespace mrrocpp {
namespace lib {
namespace smb {

action::action() :
		rotationPin(0), dThetaInd(0), dPkmTheta(0), duration(0)
{
}

//! Get motion duration parameter
double action::getDuration() const
{
	return duration;
}

//! Get PKM rotation
double action::getdPkmTheta() const
{
	return dPkmTheta;
}

//! Get rotation pin
unsigned int action::getRotationPin() const
{
	return rotationPin;
}

//! Get mobile base transrotation
int action::getdThetaInd() const
{
	return dThetaInd;
}

//! Set motion duration parameter
void action::setDuration(double duration)
{
	if (duration < 0) {
		BOOST_THROW_EXCEPTION(action_parameter_error());
	}

	this->duration = duration;
}

//! Set PKM relative rotation
void action::setdPkmTheta(double dPkmTheta)
{
	if (dPkmTheta < -2 * M_PI || dPkmTheta > 2 * M_PI) {
		BOOST_THROW_EXCEPTION(action_parameter_error());
	}

	this->dPkmTheta = dPkmTheta;
}

//! Set PIN to rotate about
void action::setRotationPin(unsigned int rotationPin)
{
	if (rotationPin > 3) {
		BOOST_THROW_EXCEPTION(action_parameter_error());
	}

	this->rotationPin = rotationPin;
}

//! Set mobile base relative rotation
void action::setdThetaInd(int dThetaInd)
{
	if (dThetaInd < -5 || dThetaInd > +5) {
		BOOST_THROW_EXCEPTION(action_parameter_error());
	}

	this->dThetaInd = dThetaInd;
}

festo_command_td::festo_command_td()
{
	for (int i = 0; i < LEG_CLAMP_NUMBER; ++i) {
		// Defaults to out...
		leg[i] = OUT;

		// Do not ask...
		undetachable[i] = false;
	}
}

motor_command::motor_command() :
		base_vs_bench_rotation(0), pkm_vs_base_rotation(0.0), estimated_time(0.0)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

