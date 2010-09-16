// -------------------------------------------------------------------------
//                            ui.h
// Definicje struktur danych i metod dla procesu UI
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __UI_SR_H
#define __UI_SR_H

#include <boost/function.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <stdexcept>
#include <iostream>
#include <string>
#include <list>

#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "ui/src/ui.h"

namespace mrrocpp {
namespace uin {
namespace common {

static const int UI_SR_BUFFER_LENGHT = 50;

class sr_buffer : public boost::noncopyable
{
private:

	Interface& interface;
	boost::thread *thread_id;
	boost::circular_buffer <lib::sr_package_t> cb;
	boost::mutex mtx; // = PTHREAD_MUTEX_INITIALIZER ;


public:

	sr_buffer(Interface& _interface);
	~sr_buffer();

	//! main thread loop
	void operator()();

	void put_one_msg(const lib::sr_package_t& new_msg); // podniesienie semafora
	void get_one_msg(lib::sr_package_t& new_msg); // podniesienie semafora
	bool buffer_empty(); // czy bufor cykliczny jest pusty
};

}
} //namespace uin
} //namespace mrrocpp
#endif

