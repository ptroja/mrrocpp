#include "RawSocket.h"

#include <stdexcept>

#include <sys/socket.h>
#include <sys/ioctl.h>
//#include <sys/ioctl_socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/*
 #include <cstdio>

 #include <iostream>
 */

namespace mrrocpp {
namespace edp {
namespace sensor {

static const uint8_t dummyMac[] = { 0, 0, 0, 0, 0, 0 };
using std::runtime_error;

static bpf_insn
		filterCommands[] = { BPF_STMT(BPF_LD + BPF_H + BPF_ABS, 12), BPF_JUMP(BPF_JMP + BPF_JEQ + BPF_K, 55555, 0, 1), BPF_STMT(BPF_RET
				+ BPF_K, -1), BPF_STMT(BPF_RET + BPF_K, 0) };

static bpf_program bpfProgram = { 4, filterCommands };

RawSocket::RawSocket(const char *device, const uint8_t mac[6])
{
	setupBpf(device, mac);
}

RawSocket::RawSocket(const char *device)
{
	setupBpf(device, dummyMac);
}

void RawSocket::setupBpf(const char *device, const uint8_t dstMac[6])
{
	char buf[11] = { 0 };

	for (int i = 0; i < 99; i++) {
		std::sprintf(buf, "/dev/bpf%i", i);
		bpf_ = open(buf, O_RDWR);
		if (bpf_ != -1) {
			break;
		}
	}

	ifreq bound_if;
	std::strcpy(bound_if.ifr_name, device);
	if (ioctl_socket(bpf_, BIOCSETIF, &bound_if) > 0) {
		throw runtime_error("Could not open device");
	}
	bpfBufLen_ = 1;

	// activate immediate mode (therefore, buf_len is initially set to "1")
	if (ioctl_socket(bpf_, BIOCIMMEDIATE, &bpfBufLen_) == -1) {
		throw runtime_error("ioctl_socket failed");
	}
	// request buffer length
	if (ioctl_socket(bpf_, BIOCGBLEN, &bpfBufLen_) == -1) {
		throw runtime_error("ioctl_socket failed");
	}
	// set promiscuous mode
	if (ioctl_socket(bpf_, BIOCPROMISC, NULL) == -1) {
		throw runtime_error("Unable to set promiscuous mode");
	}
	// capture only incoming packets
	int val = 0;
	if (ioctl_socket(bpf_, BIOCSSEESENT, &val) == -1) {
		throw runtime_error("Unable to set direction");
	}
	// turn on blocking socket
	val = 0;
	if (ioctl_socket(bpf_, FIONBIO, &val) == -1) {
		throw runtime_error("Unable to set direction");
	}
	// set filter
	if (ioctl_socket(bpf_, BIOCSETF, &bpfProgram) == -1) {
		throw runtime_error("Unable to set filter");
	}
	// Prepare send buffer
	std::memcpy(sendBuf_, dstMac, 6);
	//std::memcpy(sendBuf_ + 6, srcMac, 6);
	// Set Ether proto = 55555 (D9 03)
	sendBuf_[12] = 0xD9;
	sendBuf_[13] = 0x03;
	bpf_buf = new bpf_hdr[bpfBufLen_];
}

void RawSocket::send(const unsigned char *buf, std::size_t len)
{
	std::memcpy(sendBuf_ + 16, buf, len);
	sendBuf_[14] = static_cast <uint8_t> (len >> 8);
	sendBuf_[15] = static_cast <uint8_t> (len & 0x00FF);
	try {
		write(bpf_, sendBuf_, ETHERNET_HEADER_LEN + 2 + len);
	} catch (std::exception & e) {
		//std::cout << e.what() << std::endl;
		throw runtime_error("Error while writing to ethernet port.");
	}
}

std::size_t RawSocket::recv(unsigned char *buf, std::size_t bufLen)
{
	bpf_hdr *bpf_packet;
	unsigned char *packet;
	std::memset(bpf_buf, 0, bpfBufLen_);
	std::size_t readBytes;
	std::size_t size = 0;
	if ((readBytes = read(bpf_, bpf_buf, bpfBufLen_)) > 0) {
		// read all packets that are included in bpf_buf. BPF_WORDALIGN is used
		// to proceed to the next BPF packet that is available in the buffer.

		char *ptr = reinterpret_cast <char *> (bpf_buf);
		while (ptr < (reinterpret_cast <char *> (bpf_buf) + readBytes)) {
			bpf_packet = reinterpret_cast <bpf_hdr *> (ptr);
			packet = ((unsigned char *) bpf_packet + bpf_packet->bh_hdrlen);
			/*
			 * for (int i = 0; i < 6; ++i) {
			 * printf("%x ", packet[14 + i]);
			 * }
			 * printf("\n");
			 */
			if (!(packet[12] == 0xD9 && packet[13] == 0x03)) {
				continue;
			}
			size = (packet[14] << 8) + packet[15];
			std::memcpy(buf, packet + 16, size);
			ptr += BPF_WORDALIGN(bpf_packet->bh_hdrlen + bpf_packet->bh_caplen);
		}
	} else {
		throw runtime_error("Error while reading from bpf");
	}
	return size;
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
