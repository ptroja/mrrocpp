#ifndef FORCE_SENSOR_6284_H
#define FORCE_SENSOR_6284_H

#include <boost/cstdint.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "RawSocket.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

class ForceSensor6284 {
public:
	typedef struct _AdcReadings {
	    bool timeout;
	    int16_t readings[6];
	} AdcReadings_t;

    ForceSensor6284(RawSocket* socket);
    ~ForceSensor6284();
    AdcReadings_t getAdcReadings(boost::posix_time::time_duration timeout);
private:
    void receiveThread();
    RawSocket* socket_;
    unsigned char recvBuf_[1024];
    unsigned char sendBuf_[1024];
    boost::thread receiveThread_;
    std::size_t responseSize_;
    boost::mutex responseReceivedMt_;
    boost::condition_variable responseReceivedCv_;
};

}
}
}

#endif
