#if !defined(_TRANSMITTER_H)
#define _TRANSMITTER_H

/*!
 * @file
 * @brief File contains ecp_mp transmitter base class declaration
 * @author ptroja, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include "base/lib/impconst.h"
#include "base/lib/srlib.h"

namespace mrrocpp {
namespace ecp_mp {
namespace task {
// XXX Forward declaration
class task;
}
}
}

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

class transmitter_base
{
	// Klasa bazowa dla transmitterow (klasa abstrakcyjna)
	// Transmittery konkretne wyprowadzane sa z klasy bazowej
public:
	const lib::TRANSMITTER_t transmitter_name; // nazwa czujnika z define w impconst.h

protected:
	// Wskaznik na obiekt do komunikacji z SR
	lib::sr_ecp &sr_ecp_msg;

public:
	transmitter_base(lib::TRANSMITTER_t _transmitter_name, const std::string & _section_name, task::task& _ecp_mp_object);

	virtual ~transmitter_base()
	{
	}

	// odczyt z zawieszaniem lub bez
	virtual bool t_read(bool wait)
	{
		return true;
	}

	// zapis
	virtual bool t_write(void)
	{
		return true;
	}

};

class transmitter_error
{ // Klasa obslugi bledow czujnikow
public:
	const lib::error_class_t error_class;

	transmitter_error(lib::error_class_t err_cl);

}; // end: class transmitter_error

template <typename TO_VA, typename FROM_VA>
class transmitter : public transmitter_base
{
public:
	// Bufor nadawczy
	TO_VA to_va;
	// Bufor odbiorczy
	FROM_VA from_va;

	//! Constructor
	transmitter(lib::TRANSMITTER_t _transmitter_name, const std::string & _section_name, task::task& _ecp_mp_object) :
		transmitter_base(_transmitter_name, _section_name, _ecp_mp_object)
	{
	}
};

} // namespace transmitter


} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _TRANSMITTER_H */
