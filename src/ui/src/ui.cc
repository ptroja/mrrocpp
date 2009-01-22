/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <semaphore.h>
#include <pthread.h>

#include "lib/srlib.h"
#include "ui/ui_const.h"
#include "ui/ui.h"
// Konfigurator.
// #include "lib/configurator.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"


ui_sr_buffer::ui_sr_buffer()
{
	writer_buf_position=-1;
	reader_buf_position=-1;

	pthread_mutex_init(&mutex, NULL );

	sem_init( &sem, 0, 0);
}

int	ui_sr_buffer::set_new_msg() // podniesienie semafora
{
	sem_trywait(&sem);
	return sem_post(&sem);// odwieszenie watku edp_master
}

int	ui_sr_buffer::check_new_msg() // oczekiwanie na semafor
{
	return sem_trywait(&sem);
}

int	ui_sr_buffer::lock_mutex() // zajecie mutex'a
{
	return pthread_mutex_lock( &mutex );
}

int	ui_sr_buffer::unlock_mutex() // zwolnienie mutex'a
{
	return pthread_mutex_unlock( &mutex );
}


ui_ecp_buffer::ui_ecp_buffer()
{
	sem_init( &sem, 0, 0);
	communication_state = UI_ECP_AFTER_REPLY;
}

int	ui_ecp_buffer::post_sem() // podniesienie semafora
{
	return sem_post(&sem); // odwieszenie watku edp_master
}

int	ui_ecp_buffer::take_sem() // oczekiwanie na semafor
{
	return sem_wait(&sem);
}

int	ui_ecp_buffer::trywait_sem() // oczekiwanie na semafor
{
	return sem_trywait(&sem);
}

