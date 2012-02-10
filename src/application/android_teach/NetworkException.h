#ifndef ANDROID_EXCEPTION_H_
#define ANDROID_EXCEPTION_H_

#include "base/lib/exception.h"


namespace mrrocpp {
namespace ecp_mp {
namespace sensor {
namespace android_teach {


class NetworkException : public lib::exception::system_error
{
private:
	int code;
};









}
}
}
}


#endif
