/*	! \file src/sensor/ati6284/osiBus.cc
    * \brief NI osiBus.cpp holds the non-inlined platform
    *  independent code for the iBus
*/

//!< Platform independent headers

#include "osiBus.h"

//!< only the Physical Address attribute is supported now
//!< This is needed to initialize the MITE.  This could be used
//!< to support other attributes as well: such as Interrupt #.
u32 iBus::get(u32 attribute, u32 occurrence, tStatus *status)
{
	if(attribute == kBusAddressPhysical){
			switch(occurrence)
			{
				case    kPCI_BAR0:  return _physBar[0];
				case    kPCI_BAR1:  return _physBar[1];
				case    kPCI_BAR2:  return _physBar[2];
				case    kPCI_BAR3:  return _physBar[3];
				case    kPCI_BAR4:  return _physBar[4];
				case    kPCI_BAR5:  return _physBar[5];
			}
	}
	
	else if(attribute == kIsPciPxiBus){
		//!< if this iBus is for PCI/PXI cards, uncomment the line below
		return 1;
		
		//!< if this isBus is for PCMCIA boards which are not Cardbus,
		//!< they do not use a PCI/PXI interface, so uncomment the line below
		//!< return 0; 
	}
	
	if (status != NULL) *status = -1;
	return (u32)NULL;
}
