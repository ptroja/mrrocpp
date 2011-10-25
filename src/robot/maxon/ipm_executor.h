/*
 * ipm_executor.h
 *
 *  Created on: May 25, 2011
 *      Author: ptroja
 */

#ifndef IPM_EXECUTOR_H_
#define IPM_EXECUTOR_H_

#include <Eigen/Core>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>

#include "epos.h"
#include "base/edp/edp_exceptions.h"

namespace mrrocpp {
namespace edp {
namespace maxon {

/**
 * Interpolated profile motion mode execution thread
 */
template <int NUM_OF_MOTION_SEGMENTS, int NUM_OF_SERVOS>
struct ipm_executor
{

private:
	//! Thread id
	boost::thread tid;

public:
	//! Parameterized self type
	typedef ipm_executor <NUM_OF_MOTION_SEGMENTS, NUM_OF_SERVOS> self_t;

	//! Axes container
        boost::array <epos *, NUM_OF_SERVOS> axes;

	//! Check if there is a motion request for a given axis
	Eigen::Matrix <bool, 1, NUM_OF_SERVOS> is_moving;

	//! Position data vector
	Eigen::Matrix <double, NUM_OF_MOTION_SEGMENTS + 1, NUM_OF_SERVOS> p;

	//! Velocity data vector
	Eigen::Matrix <double, NUM_OF_MOTION_SEGMENTS + 1, NUM_OF_SERVOS> v;

	//! Time data vector
	Eigen::Matrix <double, NUM_OF_MOTION_SEGMENTS + 1, 1> t;

	//! Active command condition
	boost::condition_variable cond;

	//! Mutex related to condition variable
	boost::mutex mtx;

	//! Flag to be set by the client
	bool job_to_do;

	//! Constructor
	ipm_executor() :
		job_to_do(false)
	{
		tid = boost::thread(boost::bind(&ipm_executor::operator(), this));
	}

	//! Destructor
	~ipm_executor()
	{
		tid.interrupt();
		tid.join();
	}

private:
	//! Node to query about motion status
	int queryNodeId;

	//! Main thread routine
	void operator()()
	{
		while (true) {
			boost::unique_lock <boost::mutex> lock(mtx);

			while (!job_to_do) {
				cond.wait(lock);
			}

			//! Find the node, which is executing a motion
			queryNodeId = -1;
			for (int i = 0; i < NUM_OF_SERVOS; ++i) {
				if (is_moving(i)) {
					queryNodeId = i;
					break;
				}
			}

			if (queryNodeId == -1)
				continue;

			// Setup motion parameters
			for (size_t i = 0; i < axes.size(); ++i) {
				// Skip axes, which will be not executing a motion
				if (!is_moving(i))
					continue;

				axes[i]->setOperationMode(maxon::epos::OMD_INTERPOLATED_POSITION_MODE);
				// TODO: setup acceleration and velocity limit values
				axes[i]->clearPvtBuffer();
				for (int pnt = 0; pnt < 2; ++pnt) {
					axes[i]->setInterpolationDataRecord((int32_t) p(pnt, i), (int32_t) v(pnt, i), (uint8_t) t(pnt));
					printf("\rsend: %2d/%zu, free: %2d", pnt, i, axes[i]->getActualBufferSize());
					fflush(stdout);
				}
				printf("\n");

				const UNSIGNED16 status = axes[i]->getInterpolationBufferStatus();

				if (axes[i]->checkInterpolationBufferWarning(status)) {
					axes[i]->printInterpolationBufferStatus(status);
				}

				if (axes[i]->checkInterpolationBufferError(status)) {
					// FIXME: this should be done in a separate exception, which does not belong
					//        to the kinematics::spkm namespace.
					printf("InterpolationBufferStatus for axis %zu: 0x%04X\n", i, status);
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_pose_specification());
				}
			}

			// Start motion
			for (size_t i = 0; i < axes.size(); ++i) {
				// FIXME: this motion type should be initiated with a CAN broadcast message
				if (is_moving(i)) {
					axes[i]->startInterpolatedPositionMotion();
				}
			}

			// continuously upload the rest of the trajectory
			for (int pnt = 2; pnt < NUM_OF_MOTION_SEGMENTS + 1; ++pnt) {

				/** Wait until there is free space in the EPOS data FIFO.
				 *  Note: we check only the first axis.
				 */
				while (!epos::checkInterpolationBufferUnderflowWarning(axes[queryNodeId]->getInterpolationBufferStatus())) {
					// do nothing
				}

				for (size_t i = 0; i < axes.size(); ++i) {
					// Skip axes, which will be not executing a motion
					if (!is_moving(i))
						continue;

					// Send the data
					axes[i]->setInterpolationDataRecord((int32_t) p(pnt, i), (int32_t) v(pnt, i), (uint8_t) t(pnt));
					printf("\rsend: %2d/%zu, free: %2d", pnt, i, axes[i]->getActualBufferSize());
					fflush(stdout);
				}
			}

			// motion completed
			job_to_do = false;

			// notify the caller, which can eventually be asleep
			cond.notify_one();
		}
	}
};

} /* namespace maxon */
} /* namespace edp */
} /* namespace mrrocpp */

#endif /* IPM_EXECUTOR_H_ */
