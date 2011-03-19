#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <stdint.h>

#define SPEED B115200
#define PORT "/dev/ser1"

typedef struct {
int16_t ft[6];
} forceReadings;


uint64_t base_cycle, current_cycle, current_cycle2;

int open_port(void);
void sendBias(int fd);
forceReadings getFT(int fd);

int main (int argc, char* argv[])
{
	int uart, i,r;
	int licz=0;
	forceReadings ftxyz;

//printf("Can not qqq the port\n");
	int policy;
	struct sched_param param;
	if (pthread_getschedparam(0, &policy, &param)) {
		perror("pthread_getschedparam()");
	}
	param.sched_priority = 40;
	if (pthread_setschedparam(0, policy, &param)) {
		perror("pthread_setschedparam()");
	}

	uart=open_port();
//printf("2\n");
	tcflush(uart,TCIFLUSH);


sendBias(uart);

	while(1) {
		ftxyz=getFT(uart);
		if ((licz++)%1000==0) {
			for (i=0;i<6;i++) printf("sila %d=%d, ", i,ftxyz.ft[i]);
			printf("1: %u", base_cycle);
printf(" 2: %d", (current_cycle-base_cycle));
printf(" 3: %d", current_cycle2-current_cycle);
			printf("\n");
		}
 	}
	close(uart);
}

void sendBias(int fd)
{
		char bias='b';
		if (write(fd, &bias, 1) <0) printf("Blad zapisu\n");
}

forceReadings getFT(int fd)
{
	char query='q';
	int i,r=1;
	int current_bytes=14;

//	printf("bbb \n");
	uint8_t reads[14];
	forceReadings ftxyz;

	base_cycle = ClockCycles();

	if (write(fd, &query, 1) <0) printf("Blad zapisu\n");
current_cycle = ClockCycles();
	while ((current_bytes>0)&&(r!=0))
	{//printf("aaa: %d\n",r);
		r=read(fd, &(reads[14-current_bytes]), current_bytes);

		current_bytes-=r;
	}
current_cycle2 = ClockCycles();

	if (r==0) {
		printf("Nie otrzymano oczekiwanej ilosci znakow: %d\n",r);

	}

	for (i=0;i<6;i++) {
		ftxyz.ft[i]=(reads[2*i])<<8 | reads[2*i+1];
	}
	return ftxyz;
}

int open_port(void)
{
	int fd;
	struct termios options;
	struct termios org_port_options;
	fd = open(PORT, O_RDWR);
	if(fd == -1)
	{
		printf("Can not open the port");
		return -1;
	}
	//printf("3\n");
	tcgetattr(fd, &org_port_options);
	options = org_port_options;

	cfsetispeed(&options, SPEED);
	cfsetospeed(&options, SPEED);

	options.c_cflag |= CLOCAL; // Local line - Do not change the owner
	options.c_cflag |= CREAD; // Enable receiver
	options.c_cflag &= ~PARENB; // parity - enabled
	options.c_cflag &= ~PARODD; // Odd parity - even
	options.c_cflag &= ~CSTOPB; // stop bit - 1
	options.c_cflag &= ~CSIZE; // bit mask for data bits
	options.c_cflag |= CS8; // data bits - 8
	options.c_lflag &= ~ECHO; // Echo the Terminal screen - off
	options.c_lflag &= ~ICANON; // Canonical Input else raw - disable
	options.c_iflag &= ~INPCK; // Parity Check - disable
	options.c_iflag &= ~IGNBRK; // Parity Errors - Ignore
	options.c_iflag &= ~PARMRK; // Mark Parity Errors - disable
	options.c_iflag &= ~ISTRIP; // Strip Parity bits - disable
	options.c_iflag &= ~IXON; // Software outgoing flow control - disable
	options.c_iflag &= ~IXOFF; // Software incoming flow control - disable
	options.c_oflag &= ~OPOST; // Post process output (not set = raw output)
	options.c_cc[VTIME] = 1;
		options.c_cc[VMIN] = 0;
	tcsetattr(fd, TCSADRAIN, &options);
	fcntl(fd, F_SETFL, 0);
//	printf("4\n");
	return fd;
}
