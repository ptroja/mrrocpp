////////////////////////////////////////////////////////////////////////////////
/*! \file     src/lib/com_buf.h
 *
 *  Data structures for IPC.
 *
 *  \author   tkornuta
 *  \date     2006-11-29
 *  \URL: https://segomo.elka.pw.edu.pl/svn/mrrocpp/base/trunk/include/lib/com_buf.h $
 *  $LastChangedRevision: 4088 $
 *  $LastChangedDate: 2010-04-13 12:53:09 +0200 (Tue, 13 Apr 2010) $
 *  $LastChangedBy: yoyek $
 *
 *  \todo <ul>
 *          <li>Translate to English where necessary.</li>
 *          <li>Write detailed comments.</li>
 *          <li>Suplement comments for those consts, variables and structures
 *              that are not commented at all.</li>
 *          <li>Clean up the commented fragments of code.</li>
 *        </ul>
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef __SWARMITFIX_H
#define __SWARMITFIX_H


namespace mrrocpp {
namespace lib {



enum EPOS_GEN_PROFILE {
	TRAPEZOIDAL_VELOCITY=0, CUBIC_POSITION, EPOS_GEN_PROFILE_NO_ACTION
};

enum SMB_PIN_INSERTION {
	INSERT = 0, WITHDRAWN, SMB_PIN_INSERTION_NO_ACTION
}; // namespace mrrocpp

enum SMB_PIN_LOCKING {
	CLAMB = 0, UNCLAMB, SMB_PIN_LOCKING_NO_ACTION
}; // namespace mrrocpp

enum SHEAD_HEAD_SOLIDIFICATION {
	SOLIDIFY = 0, DESOLIDIFY, SHEAD_HEAD_SOLIDIFICATION_NO_ACTION
}; // namespace mrrocpp

enum SHEAD_VACUUM_ACTIVATION {
	VACUUM_ON= 0, VACUUM_OFF, SHEAD_VACUUM_ACTIVATION_NO_ACTION
}; // namespace mrrocpp

}
}


#endif
