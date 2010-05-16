#include "RawSocket.h"
#include "ForceSensor.h"
#include <stdexcept>

namespace mrrocpp {
namespace edp {
namespace sensor {


ForceSensor6284::ForceSensor6284(RawSocket & socket) : socket_(socket) {
    sendBuf_[0] = 0;
    sendBuf_[1] = 0;
    receiveThread_ = boost::thread(&ForceSensor6284::receiveThread, this);
}

ForceSensor6284::~ForceSensor6284() {
    receiveThread_.join();
}


ForceSensor6284::AdcReadings_t ForceSensor6284::getAdcReadings(boost::posix_time::time_duration timeout) {
    responseSize_ = 0;
    // TODO: why we send 4 instead of 2 bytes (sending uninitialized values)?
    socket_.send(sendBuf_, 4);
    boost::system_time currentTime = boost::get_system_time();
    boost::system_time expTime = currentTime + timeout;

    AdcReadings_t readings;
    readings.timeout = false;

    boost::unique_lock<boost::mutex> lock(responseReceivedMt_);
    while (responseSize_ == 0) {
        if (!responseReceivedCv_.timed_wait(lock, expTime)) {
            readings.timeout = true;
            return readings;
        }
    }
    std::memcpy(readings.readings, recvBuf_ + 2, 12);
    return readings;
}

void ForceSensor6284::receiveThread() {
    while (true) {
    	uint8_t localRecvBuf[1024];
    	std::size_t bytesReceived = socket_.recv(recvBuf_, sizeof(recvBuf_));
        if (bytesReceived == 0) {
            break;
        }
	    if (bytesReceived < 0) {
            throw std::runtime_error("Unable to receive from raw socket");
	    } else {
	    	// lock the shared data
            boost::lock_guard<boost::mutex> lock(responseReceivedMt_);

            // copy values to shared data
            std::memcpy(recvBuf_, localRecvBuf, bytesReceived);
            responseSize_ = bytesReceived;

            // notify the waiting thread
	        responseReceivedCv_.notify_one();
	    }
    }
}

}
}
}
