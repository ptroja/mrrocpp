#include "bird_hand.h"
#include "combuf.h"

#include <exception>
#include <stdexcept>
#include <string.h>

#define BAUD 921600

void Bird_hand::write_read(int fd, char* b, unsigned int w_len, unsigned int r_len)
{
	unsigned int dlen = 0;
	fd_set rfds;

	write(fd, b, w_len);

	while (dlen < r_len) {
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);

		// timeout
		struct timeval timeout;
		timeout.tv_sec = (time_t) 10;
		timeout.tv_usec = 500;

		int select_retval = select(fd + 1, &rfds, NULL, NULL, &timeout);

		if (select_retval == 0) {
			throw(std::runtime_error("communication timeout !!!"));
		} else {
			dlen += read(fd, b + dlen, r_len - dlen);
		}
	}

}

Bird_hand::Bird_hand()
{

}

Bird_hand::~Bird_hand()
{

}

void Bird_hand::connect(std::string port)
{
	for (unsigned int i = 0; i < 8; i++) {
		fd[i] = open((port+(char)(i+48)).c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd[i] < 0) {
			throw(std::runtime_error("unable to open device device !!!"));
		}

		tcgetattr(fd[i], &oldtio[i]);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CS8 | CLOCAL | CREAD;
		newtio.c_iflag = INPCK; //IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0) {
			tcsetattr(fd[i], TCSANOW, &oldtio[i]);
			close(fd[i]);
			fd[i] = -1;
			throw(std::runtime_error("unable to set baudrate device !!!"));
			return;
		}
		// activate new settings
		tcflush(fd[i], TCIFLUSH);
		tcsetattr(fd[i], TCSANOW, &newtio);
	}
}

void Bird_hand::disconnect()
{
	for (unsigned int i = 0; i < 8; i++) {
		if (fd[i] > 0) {
			tcsetattr(fd[i], TCSANOW, &oldtio[i]);
			close(fd[i]);
		}
	}

}

void Bird_hand::getStatus(uint8_t id, uint8_t &status, int32_t &position, int16_t &current, int16_t &torque)
{
	buf[0] = START_BYTE;
	buf[1] = id % 2;
	buf[2] = GET_STATUS;

	write_read(fd[id / 2], buf, 21, (HEADER_LEN + sizeof(struct status_)));

	if (buf[0] != '#')
		printf("error \n");

	struct status_* stat = (status_*) &buf[3];

	status = stat->status;
	position = stat->position;
	current = stat->current;
	torque = stat->force;
}

void Bird_hand::getSynchroPos(uint8_t id, int16_t &pos)
{

	buf[0] = '#';
	buf[1] = id % 2;
	buf[2] = GET_ABSPOS;

	write_read(fd[id / 2], buf, 21, (HEADER_LEN + sizeof(struct abspos_)));

	struct abspos_ *a = (struct abspos_*) &buf[3];

	pos = a->abspos;

}

void Bird_hand::getPID(uint8_t id, int16_t &p, int16_t &i, int16_t &d)
{

	buf[0] = '#';
	buf[1] = id % 2;
	buf[2] = GET_PID;

	write_read(fd[id / 2], buf, 21, (HEADER_LEN + sizeof(struct pid_)));

	struct pid_ *a = (struct pid_*) &buf[3];

	p = a->p;
	i = a->i;
	d = a->d;

}

void Bird_hand::setPID(uint8_t id, int16_t p, int16_t i, int16_t d)
{

	buf[0] = '#';
	buf[1] = id % 2;
	buf[2] = SET_PID;

	struct pid_ *a = (struct pid_*) &buf[3];

	a->p = p;
	a->i = i;
	a->d = d;

	write_read(fd[id / 2], buf, 21, 0);
}

void Bird_hand::getLimit(uint8_t id, int16_t &upper, int16_t &lower)
{

	buf[0] = '#';
	buf[1] = id % 2;
	buf[2] = GET_LIMIT;

	write_read(fd[id / 2], buf, 21, (HEADER_LEN + sizeof(struct limit_)));

	struct limit_ *a = (struct limit_*) &buf[3];

	upper = a->u_limit;
	lower = a->l_limit;

}

void Bird_hand::setLimit(uint8_t id, int16_t upper, int16_t lower)
{

	buf[0] = '#';
	buf[1] = id % 2;
	buf[2] = SET_LIMIT;

	struct limit_ *a = (struct limit_*) &buf[3];

	a->u_limit = upper;
	a->l_limit = lower;

	write_read(fd[id / 2], buf, 21, 0);

}

void Bird_hand::setCMD1(uint8_t id, int16_t t, int16_t b, int16_t Fd, int32_t rd)
{
	buf[0] = '#';
	buf[1] = id % 2;
	buf[2] = SET_CMD1;

	struct cmd_ *x = (struct cmd_ *) &buf[3];
	x->t = t;
	x->fd = Fd;
	x->b = b;
	x->rd = rd;

	write_read(fd[id / 2], buf, 21, 0);
}

void Bird_hand::setCMD2(uint8_t id, int16_t t, int16_t b, int16_t Fd, int32_t rd)
{
	buf[0] = '#';
	buf[1] = id % 2;
	buf[2] = SET_CMD2;

	struct cmd_ *x = (struct cmd_ *) &buf[3];
	x->t = t;
	x->fd = Fd;
	x->b = b;
	x->rd = rd;

	write_read(fd[id / 2], buf, 21, 0);
}

void Bird_hand::setCMD3(uint8_t id, int16_t t, int16_t b, int16_t Fd, int32_t rd)
{
	buf[0] = '#';
	buf[1] = id % 2;
	buf[2] = SET_CMD3;

	struct cmd_ *x = (struct cmd_ *) &buf[3];
	x->t = t;
	x->fd = Fd;
	x->b = b;
	x->rd = rd;

	write_read(fd[id / 2], buf, 21, 0);
}

void Bird_hand::synchronize(uint8_t id, uint16_t step)
{
	buf[0] = '#';
	buf[1] = id;
	buf[2] = SET_SYNCHRO;

	struct synchro_ *x = (struct synchro_ *) &buf[3];
	x->n = step;
	for (unsigned int i = 0; i < 4; i++)
		write_read(fd[i], buf, 21, 0);

}
