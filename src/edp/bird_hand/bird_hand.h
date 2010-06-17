
#include <inttypes.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

#include <string>

/*!
 * \brief class of EDP bird hand gripper hardware interface.
 *
 * This class provides methods for communication with bird hand hardware controllers.
 */
class Bird_hand
{
	public:
	Bird_hand();
	~Bird_hand();

	/*!
	 * \brief method connect to hardware controller.
	 */
	void connect(std::string port);

	/*!
	 * \brief method disconnect from hardware controller.
	 */
	void disconnect();
	
	/*!
	 * \brief method get synchronization position.
	 *
	 * Get initial position of gripper joint.
	 */
	void getSynchroPos(uint8_t id, int16_t &pos);
	
	/*!
	 * \brief method get status.
	 *
	 * Get actual status of gripper joint.
	 */
	void getStatus(uint8_t id, uint8_t &status, int32_t &position, int16_t &current, int16_t &torque);
	
	/*!
	 * \brief method get PID parameters.
	 *
	 * Get PID regulator parameters of gripper joint.
	 */
	void getPID(uint8_t id, int16_t &p, int16_t &i, int16_t &d);

	/*!
	 * \brief method set PID parameters.
	 *
	 * Set PID regulator parameters of gripper joint.
	 */
	void setPID(uint8_t id, int16_t p, int16_t i, int16_t d);
	
	/*!
	 * \brief method get limits parameters.
	 *
	 * Get limits parameters of gripper joint.
	 */
	void getLimit(uint8_t id, int16_t &upper, int16_t &lower);

	/*!
	 * \brief method get limits parameters.
	 *
	 * Set limits parameters of gripper joint.
	 */
	void setLimit(uint8_t id, int16_t upper, int16_t lower);
	
	/*!
	 * \brief method set command for gripper joint.
	 *
	 * Set command for gripper joint in velocity mode.
	 */
	void setCMD1(uint8_t id, int16_t t, int16_t b, int16_t Fd, int32_t rd);

	/*!
	 * \brief method set command for gripper joint.
	 *
	 * Set command for gripper joint in relative position mode.
	 */
	void setCMD2(uint8_t id, int16_t t, int16_t b, int16_t Fd, int32_t rd);

	/*!
	 * \brief method set command for gripper joint.
	 *
	 * Set command for gripper joint in absolut position mode.
	 */
	void setCMD3(uint8_t id, int16_t t, int16_t b, int16_t Fd, int32_t rd);
	
	/*!
	 * \brief method time synchronization.
	 *
	 * Time synchronization off all joints.
	 * id must be set to 255.
	 */
	void synchronize(uint8_t id, uint16_t step);

	protected:
	private:
	
	void write_read(int fd, char* buf, unsigned int w_len, unsigned int r_len);
	
	int fd[8];
	struct termios oldtio[8];

	char buf[30];

};
