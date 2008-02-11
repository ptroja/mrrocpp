#if !defined(_ECP_GEN_PLAYERPOS_H)
#define _ECP_GEN_PLAYERPOS_H

#include "ecp/common/ecp_generator.h"

class playerpos_generator : public ecp_generator 
{

protected:
	
public:
	// konstruktor
	playerpos_generator(ecp_task& _ecp_task);
	
	virtual bool first_step ();

	virtual bool next_step ();

}; // end:

#endif /* _ECP_GEN_PLAYERPOS_H */
