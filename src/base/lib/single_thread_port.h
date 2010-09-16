#ifndef SINGLE_THREAD_PORT_H_
#define SINGLE_THREAD_PORT_H_

/*!
 * @file
 * @brief File contains template class declaration for a single thread communication ports
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup lib
 */

#include <string>
#include <map>

#include <boost/cast.hpp>

namespace mrrocpp {
namespace lib {

class single_thread_port_manager;

/*!
 * @brief Data flow status
 *
 * @ingroup lib
 */
enum FlowStatus
{
	NoData, OldData, NewData
};

/*!
 * @brief single_thread_port interface class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup lib
 */
class single_thread_port_interface
{
private:
	/**
	 * @brief Unique port name
	 */
	const std::string name;

protected:
	/**
	 * @brief new data flag
	 */
	bool new_data;

public:
	/**
	 * @brief Constructor
	 * @param _name Unique port name.
	 * @param _port_manager port manager reference.
	 */
	single_thread_port_interface(std::string _name, single_thread_port_manager & _port_manager);
	/**
	 * @brief Destructor
	 * This is a base class, so virtual destructor is recommended
	 * and it is also required for dynamic casting.
	 */
	virtual ~single_thread_port_interface()
	{
	}

	/**
	 * @brief returns port name
	 */
	std::string get_name();

	/**
	 * @brief clears all flags
	 */
	virtual void clear_all_flags()=0;

};

/*!
 * @brief template class of communication port for single thread usage
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup lib
 */
template <class T>
class single_thread_port : public single_thread_port_interface
{
protected:
	/**
	 * @brief no data flag
	 * it is set if no data is stored (before first set method call)
	 */
	bool no_data;

public:
	/**
	 * @brief data stored basing on template
	 */
	T data;

	/**
	 * @brief Constructor
	 * @param _name Unique port name.
	 * @param _port_manager port manager reference.
	 */
	single_thread_port(std::string _name, single_thread_port_manager & _port_manager) :
		single_thread_port_interface(_name, _port_manager), no_data(true)

	{
	}

	/**
	 * @brief Sets the new_data flag and unset no_data flag
	 */
	virtual void set()
	{
		no_data = false;
		new_data = true;
	}

	/**
	 * @brief returns Flow status
	 */
	virtual FlowStatus get()
	{
		if (no_data) {
			return NoData;
		} else if (new_data) {
			new_data = false;
			return NewData;
		} else {
			return OldData;
		}
	}

	/**
	 * @brief clears new_data flag
	 */
	void clear_new_data_flag()
	{
		new_data = false;
	}

	/**
	 * @brief clears all flags
	 */
	virtual void clear_all_flags()
	{
		clear_new_data_flag();
	}

	/**
	 * @brief test method for test purposes
	 */
	void test()
	{

	}

};

/*!
 * @brief template class of communication request port for single thread usage
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup lib
 */
template <class T>
class single_thread_request_port : public single_thread_port <T>
{
protected:
	/**
	 * @brief new request flag
	 */
	bool new_request;

public:

	/**
	 * @brief Constructor
	 * @param _name Unique port name.
	 * @param _port_manager port manager reference.
	 */
	single_thread_request_port(std::string _name, single_thread_port_manager & _port_manager) :
		single_thread_port <T> (_name, _port_manager), new_request(false)

	{
	}

	/**
	 * @brief sets new request flag
	 */
	void set_request()
	{
		new_request = true;
	}

	/**
	 * @brief calls set method of single_thread_port to set new_data flag
	 */
	void set()
	{
		new_request = false;
		single_thread_port <T>::set();
	}

	/**
	 * @brief clears all flags
	 */
	void clear_all_flags()
	{
		single_thread_port <T>::clear_new_data_flag();
		clear_new_request_flag();
	}

	/**
	 * @brief returns new_request flag
	 */
	bool is_new_request() const

	{
		return new_request;
	}

	/**
	 * @brief clears new_request flag
	 */
	void clear_new_request_flag()
	{
		new_request = false;
	}
};

/*!
 * @brief single_thread_port_interface stl map typedef
 *
 * @ingroup lib
 */
typedef std::map <std::string, single_thread_port_interface *> single_thread_port_interface_t;

/*!
 * @brief single_thread_port_interface stl map value_type typedef
 *
 * @ingroup lib
 */
typedef single_thread_port_interface_t::value_type single_thread_port_interface_pair_t;

/*!
 * @brief class to manage single_thread_port classes
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup lib
 */
class single_thread_port_manager
{
private:
	/**
	 * @brief single_thread_port map
	 */
	single_thread_port_interface_t single_thread_port_map;

public:
	/**
	 * @brief Constructor
	 */
	single_thread_port_manager();

	/**
	 * @brief adds new port to port map
	 * @param single_thread_port_inter port interface to add
	 */
	void add_port(single_thread_port_interface* single_thread_port_inter);

	/**
	 * @brief clears all flags of stored ports
	 */
	void clear_data_ports();

	/**
	 * @brief returns single_thread_port of given name
	 * @param name port name
	 */
	template <class T>
	single_thread_port <T>* get_port(const std::string & name)
	{
		return boost::polymorphic_cast <single_thread_port <T> *>(single_thread_port_map[name]);
	}

	/**
	 * @brief returns single_thread_request_port of given name
	 * @param name port name
	 */
	template <class T>
	single_thread_request_port <T>* get_request_port(const std::string & name)
	{
		return boost::polymorphic_cast <single_thread_request_port <T> *>(single_thread_port_map[name]);
	}
};

}
}

#endif /* SINGLE_THREAD_PORT_H_ */
