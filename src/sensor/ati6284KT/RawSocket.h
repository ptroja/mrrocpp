#ifndef __RAW_SOCKET__H
#define __RAW_SOCKET__H

#include <string>
#include <cstdlib>
#include <cstring>
#include <stdint.h>

#include <csignal>
#include <net/if_dl.h>
#include <net/if_ether.h>
#include <net/bpf.h>

namespace mrrocpp {
namespace edp {
namespace sensor {

  class RawSocket {
  private:
    static const std::size_t SEND_BUF_LEN = 1024;
    static const std::size_t RECV_BUF_LEN = 1024;
    static const unsigned int ETHERNET_HEADER_LEN = 14;
  public:
     RawSocket(const char *device, const uint8_t mac[6]);
     RawSocket(const char *device);
     void send(const unsigned char *buf, std::size_t len);
     std::size_t recv(unsigned char *buf, std::size_t bufLen);
  private:
    int bpf_;
    int bpfBufLen_;
    bpf_hdr *bpf_buf;
    unsigned char sendBuf_[SEND_BUF_LEN];
    unsigned char recvBuf_[RECV_BUF_LEN];
    void setupBpf(const char *device, const uint8_t mac[6]);
//    ~RawSocket(){*this = NULL;};
  };

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

#endif
