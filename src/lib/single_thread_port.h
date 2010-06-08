/**
 * \file single_thread_port.h
 *
 * \date 2010
 * \author yoyek
 *
 * \brief Template class for single robot communication port
 */

#ifndef SINGLE_THREAD_PORT_H_
#define SINGLE_THREAD_PORT_H_

#include <string>
#include <map>

namespace mrrocpp {
namespace lib {

enum FlowStatus
{
	NoData = 0, OldData = 1, NewData = 2
};

class single_thread_port_interface
{
private:
	std::string name;

protected:
	bool new_data;

public:

	single_thread_port_interface(std::string _name) :
		name(_name), new_data(false)
	{
	}

	std::string get_name()
	{
		return name;
	}

	//virtual void test() = 0;

};

template <class T>
class single_thread_port : public single_thread_port_interface
{

protected:
	bool no_data;
	T data;

public:
	single_thread_port(std::string _name) :
		no_data(true), single_thread_port_interface(_name)
	{
	}

	virtual void set(const T& _data)
	{
		data = _data;
		no_data = false;
		new_data = true;
	}

	virtual FlowStatus get(T& _data)
	{
		if (no_data) {
			return NoData;
		} else if (new_data) {
			_data = data;
			new_data = false;
			return NewData;
		} else {
			_data = data;
			return OldData;
		}
	}

	void clear_new_data_flag()
	{
		new_data = false;
	}

	void test()
	{

	}

};

template <class T>
class single_thread_request_port : public single_thread_port <T>
{

protected:
	bool new_request;

public:
	single_thread_request_port(std::string _name) :
		single_thread_port <T> (_name), new_request(false)
	{
	}

	void set_request()
	{
		new_request = true;
	}

	void set(T& _data)
	{
		new_request = false;
		single_thread_port <T>::set(_data);
	}

	bool is_new_request()
	{
		return new_request;
	}

	void clear_new_request_flag()
	{
		new_request = false;
	}

};

class single_thread_port_manager
{
private:
	std::map <std::string, single_thread_port_interface *> single_thread_port_map;

public:

	single_thread_port_manager()
	{
	}

	void add_port(single_thread_port_interface* single_thread_port_inter)
	{
		single_thread_port_map[single_thread_port_inter->get_name()] = single_thread_port_inter;
	}

	template <class T>
	single_thread_port <T>* get_port(std::string name)
	{
		// TODO: dodac obsluge wyjatku w sytuacji gdy nie ma takiego pola lub typ sie nie zgadza
		return (single_thread_port <T>*)(single_thread_port_map[name]);
	}

	template <class T>
	single_thread_request_port <T>* get_request_port(std::string name)
	{
		// TODO: dodac obsluge wyjatku w sytuacji gdy nie ma takiego pola lub typ sie nie zgadza
		return (single_thread_request_port <T>*)(single_thread_port_map[name]);
	}

};

}
}

#endif /* SINGLE_THREAD_PORT_H_ */
