#ifndef __UI_SR_H
#define __UI_SR_H

#include <QMainWindow>

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <stdexcept>
#include <iostream>
#include <string>
#include <list>

#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"
#include "base/lib/condition_synchroniser.h"

namespace mrrocpp {
namespace ui {
namespace common {

class sr_buffer : public boost::noncopyable
{
private:
	Interface& interface;
	boost::thread thread_id;
	boost::circular_buffer <lib::sr_package_t> cb;
	boost::circular_buffer <lib::sr_package_t> cb_inter;
	boost::mutex mtx;

public:

	static const int UI_SR_BUFFER_LENGHT = 200;

	lib::condition_synchroniser thread_started;
	sr_buffer(Interface& _interface);

	~sr_buffer();

	//! main thread loop
	void operator()();

	void put_one_msg(const lib::sr_package_t& new_msg); // podniesienie semafora
	void get_one_msg(lib::sr_package_t& new_msg); // podniesienie semafora
	void inter_get_one_msg(lib::sr_package_t& new_msg); // podniesienie semafora

	bool buffer_empty(); // czy bufor cykliczny jest pusty
	bool inter_buffer_empty(); // czy bufor cykliczny jest pusty

	void copy_buffers();
	void clear_buffer();

};

}
} //namespace ui
} //namespace mrrocpp
#endif

