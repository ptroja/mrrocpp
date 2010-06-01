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

#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/mis_fun.h"

#include "ui/ui.h"

#define UI_SR_BUFFER_LENGHT 50

class ui_sr_buffer {
private:

	Ui& ui;

	boost::circular_buffer<lib::sr_package_t> cb;
	boost::mutex mtx; // = PTHREAD_MUTEX_INITIALIZER ;


public:

	ui_sr_buffer(Ui& _ui);

	void put_one_msg(const lib::sr_package_t& new_msg); // podniesienie semafora
	void get_one_msg(lib::sr_package_t& new_msg); // podniesienie semafora
	bool buffer_empty(); // czy bufor cykliczny jest pusty
};

#endif

