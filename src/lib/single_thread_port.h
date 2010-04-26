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

class single_thread_port_interface {
private:
	std::string name;

protected:
	bool new_data;

public:
	virtual bool is_new_data() = 0;

	single_thread_port_interface(std::string _name) :
		name(_name), new_data(false) {
	}

	std::string get_name() {
		return name;
	}

};

template<class T>
class single_thread_port: public single_thread_port_interface {

protected:
	T data;

public:
	single_thread_port(std::string _name) :
		single_thread_port_interface(_name) {
	}

	virtual void set(T& _data) {
		data = _data;
		new_data = true;
	}

	virtual T get() {
		new_data = false;
		return data;
	}

	bool is_new_data() {
		return new_data;
	}

};

template<class T>
class single_thread_request_port: public single_thread_port<T> {

protected:
	bool new_request;

public:
	single_thread_request_port(std::string _name) :
		single_thread_port<T> (_name) {
	}

	void set_request() {
		new_request = true;
	}

	void set(T& _data) {
		new_request = false;
		single_thread_port<T>::set(_data);
	}

	bool is_new_request() {
		return new_request;
	}

	T get() {
		// dodac wyjatek jak nie ma nowych danych
		return single_thread_port<T>::get();
	}

};

class single_thread_port_manager {
private:
	std::map<std::string, single_thread_port_interface *>
			single_thread_port_map;

public:

	single_thread_port_manager() {
	}

	void add_port(single_thread_port_interface* single_thread_port_inter) {
		single_thread_port_map[single_thread_port_inter->get_name()]
				= single_thread_port_inter;
	}

	template<class T>
	single_thread_port<T>* get_port(std::string name) {
		// TODO: dodac obsluge wyjatku w sytuacji gdy nie ma takiego pola lub typ sie nie zgadza
		return dynamic_cast<single_thread_port<T>*> (single_thread_port_map[name]);
	}

	template<class T>
	single_thread_request_port<T>* get_request_port(std::string name) {
		// TODO: dodac obsluge wyjatku w sytuacji gdy nie ma takiego pola lub typ sie nie zgadza
		return dynamic_cast<single_thread_request_port<T>*> (single_thread_port_map[name]);
	}

};

}
}

#endif /* SINGLE_THREAD_PORT_H_ */
