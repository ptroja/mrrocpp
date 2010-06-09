#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"				// numery bledow
#include "lib/srlib.h"					// klasy bledow
#include "application/playerjoy/ecp_mp_tr_player.h"
#include "player/playerc.h"

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

player::player  (
    TRANSMITTER_ENUM _transmitter_name, const char* _section_name, task::task& _ecp_mp_object,
    const char *host, unsigned int port,
    const char *devname, int devindex, int access
)  :
	transmitter (_transmitter_name, _section_name, _ecp_mp_object)
{

	int err;
	err = pthread_cond_init(&this->cond, NULL);
	if (err) {
		fprintf(stderr, "player_transmitter::player_transmitter(): pthread_cond_init(): %s\n",
		        strerror(err));
	}
	err = pthread_mutex_init(&this->mtx, NULL);
	if (err) {
		fprintf(stderr, "player_transmitter::player_transmitter(): pthread_mutex_init(): %s\n",
		        strerror(err));
	}

	client = playerc_client_create(NULL, host, port);

	printf("1\n");
	if (playerc_client_connect(client) != 0) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return;
	}

	printf("2\n");
	if ((if_code = playerc_lookup_code(devname)) < 0) {
		fprintf(stderr, "unknown Player device name\n");
		return;
	}

	switch (if_code) {
		case PLAYER_JOYSTICK_CODE:              // Joytstick
			device = (playerc_device_t *) playerc_joystick_create(client, devindex);
			fprintf(stderr, "playerc_joystick_created\n");
			break;
		case PLAYER_POSITION_CODE:              // device that moves about
			device = (playerc_device_t *) playerc_position_create(client, devindex);
			fprintf(stderr, "playerc_position_created\n");
			break;
		case PLAYER_PLAYER_CODE:                // the server itself
		case PLAYER_POWER_CODE:                 // power subsystem
		case PLAYER_GRIPPER_CODE:               // gripper
		case PLAYER_SONAR_CODE:                 // fixed range-finder
		case PLAYER_LASER_CODE:                 // scanning range-finder
		case PLAYER_BLOBFINDER_CODE:            // visual blobfinder
		case PLAYER_PTZ_CODE:                   // pan-tilt-zoom unit
		case PLAYER_AUDIO_CODE:                 // audio I/O
		case PLAYER_FIDUCIAL_CODE:              // fiducial detector
		case PLAYER_SPEECH_CODE:                // speech I/O
			device = (playerc_device_t *) playerc_speech_create(client, devindex);
			fprintf(stderr, "playerc_speech_created\n");
			break;
		case PLAYER_GPS_CODE:                   // GPS unit
		case PLAYER_BUMPER_CODE:                // bumper array
		case PLAYER_TRUTH_CODE:
		case PLAYER_IDARTURRET_CODE:            // ranging + comms
		case PLAYER_IDAR_CODE:                  // ranging + comms
		case PLAYER_DESCARTES_CODE:             // the Descartes platform
		case PLAYER_DIO_CODE:                   // digital I/O
		case PLAYER_AIO_CODE:                   // analog I/O
		case PLAYER_IR_CODE:                    // IR array
		case PLAYER_WIFI_CODE:                  // wifi card status
		case PLAYER_WAVEFORM_CODE:              // fetch raw waveforms
		case PLAYER_LOCALIZE_CODE:              // localization
		case PLAYER_MCOM_CODE:                  // multicoms
		case PLAYER_SOUND_CODE:                 // sound file playback
		case PLAYER_AUDIODSP_CODE:              // audio dsp I/O
		case PLAYER_AUDIOMIXER_CODE:            // audio I/O
		case PLAYER_POSITION3D_CODE:            // 3-D position
		case PLAYER_SIMULATION_CODE:            // simulators
		case PLAYER_SERVICE_ADV_CODE:           // LAN service advertisement
		case PLAYER_BLINKENLIGHT_CODE:          // blinking lights
		case PLAYER_NOMAD_CODE:                 // Nomad robot
		case PLAYER_CAMERA_CODE:
		case PLAYER_MAP_CODE:                   // get a map
		case PLAYER_PLANNER_CODE:               // 2D motion planner
		case PLAYER_LOG_CODE:                   // log read/write control
		case PLAYER_ENERGY_CODE:                // energy consumption & charging
		case PLAYER_MOTOR_CODE:                 // motor interface
		case PLAYER_POSITION2D_CODE:            // 2-D position
		case PLAYER_SPEECH_RECOGNITION_CODE:    // speech recognitionI/O
			device = (playerc_device_t *) playerc_speech_recognition_create(client, devindex);
			fprintf(stderr, "playerc_speech_recognition_created\n");
			break;
		case PLAYER_OPAQUE_CODE:                // plugin interface
		default:
			break;
	}

	printf("3.1\n");
	if (device == NULL) {
		fprintf(stderr, "Player proxy creation error\n");
		return;
	}

	printf("drivername: %s\n", device->drivername);
	printf("3\n");
	if (playerc_device_subscribe(device, access) < 0) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return;
	}
	printf("4\n");

	printf("pthread_create()...");
	fflush(stdout);
	pthread_create(&worker, NULL, query_loop, this);
	printf("done\n");
}


player::~player ()
{
	// Shutdown and tidy up
	printf("Player transmitter shutting down...");
	fflush(stdout);
	pthread_cancel(worker);
	if(pthread_join(worker, NULL)) {
		perror("Player:pthread_cancel()");
	}
	playerc_device_unsubscribe(device);
	playerc_device_term(device);
	playerc_client_disconnect(client);
	playerc_client_destroy(client);

	pthread_mutex_destroy(&this->mtx);
	pthread_cond_destroy(&this->cond);
	printf("done\n");
}

bool player::t_write()
{
	return true;
}

bool player::t_read(bool wait)
{
	if (wait) {
		pthread_mutex_lock(&this->mtx);
		int rc = 0;
		while (device->fresh == 0 && rc == 0) {
			//fprintf(stderr, "pthread_cond_wait()...");
			//fflush(stderr);
			rc = pthread_cond_wait(&this->cond, &this->mtx);
			//fprintf(stderr, "\n");
		}
	}

	switch (if_code) {
		case PLAYER_JOYSTICK_CODE:              // Joystick
			{
				playerc_joystick_t *dev = (playerc_joystick_t *) device;
				from_va.player_joystick = *dev;
			}
			break;
		case PLAYER_POSITION_CODE:              // device that moves about
			{
				playerc_position_t *dev = (playerc_position_t *) device;
				from_va.player_position = *dev;
			}
			break;
		case PLAYER_PLAYER_CODE:                // the server itself
		case PLAYER_POWER_CODE:                 // power subsystem
		case PLAYER_GRIPPER_CODE:               // gripper
		case PLAYER_SONAR_CODE:                 // fixed range-finder
		case PLAYER_LASER_CODE:                 // scanning range-finder
		case PLAYER_BLOBFINDER_CODE:            // visual blobfinder
		case PLAYER_PTZ_CODE:                   // pan-tilt-zoom unit
		case PLAYER_AUDIO_CODE:                 // audio I/O
		case PLAYER_FIDUCIAL_CODE:              // fiducial detector
		case PLAYER_SPEECH_CODE:                // speech I/O
		case PLAYER_GPS_CODE:                   // GPS unit
		case PLAYER_BUMPER_CODE:                // bumper array
		case PLAYER_TRUTH_CODE:
		case PLAYER_IDARTURRET_CODE:            // ranging + comms
		case PLAYER_IDAR_CODE:                  // ranging + comms
		case PLAYER_DESCARTES_CODE:             // the Descartes platform
		case PLAYER_DIO_CODE:                   // digital I/O
		case PLAYER_AIO_CODE:                   // analog I/O
		case PLAYER_IR_CODE:                    // IR array
		case PLAYER_WIFI_CODE:                  // wifi card status
		case PLAYER_WAVEFORM_CODE:              // fetch raw waveforms
		case PLAYER_LOCALIZE_CODE:              // localization
		case PLAYER_MCOM_CODE:                  // multicoms
		case PLAYER_SOUND_CODE:                 // sound file playback
		case PLAYER_AUDIODSP_CODE:              // audio dsp I/O
		case PLAYER_AUDIOMIXER_CODE:            // audio I/O
		case PLAYER_POSITION3D_CODE:            // 3-D position
		case PLAYER_SIMULATION_CODE:            // simulators
		case PLAYER_SERVICE_ADV_CODE:           // LAN service advertisement
		case PLAYER_BLINKENLIGHT_CODE:          // blinking lights
		case PLAYER_NOMAD_CODE:                 // Nomad robot
		case PLAYER_CAMERA_CODE:
		case PLAYER_MAP_CODE:                   // get a map
		case PLAYER_PLANNER_CODE:               // 2D motion planner
		case PLAYER_LOG_CODE:                   // log read/write control
		case PLAYER_ENERGY_CODE:                // energy consumption & charging
		case PLAYER_MOTOR_CODE:                 // motor interface
		case PLAYER_POSITION2D_CODE:            // 2-D position
		case PLAYER_SPEECH_RECOGNITION_CODE:    // speech recognitionI/O
			{
				playerc_speech_recognition_t *dev = (playerc_speech_recognition_t *) device;
				from_va.player_speech_recognition = *dev;
			}
			break;
		case PLAYER_OPAQUE_CODE:                // plugin interface
		default:
			break;
	}

	if (wait) {
		pthread_mutex_unlock(&this->mtx);
	}

	device->fresh = 0;

	return 1;
}

void * player::query_loop(void * arg)
{
	// Read data from the server
	player *me = (player *) arg;
	playerc_client_t *clnt = me->client;

	while(1) {
		int rc;
		playerc_client_read(clnt);
		//fprintf(stderr, "playerc_client_read()\n");

		if ((rc = pthread_mutex_lock(&me->mtx))) {
			fprintf(stderr, "player_transmitter::query_loop(): pthread_mutex_lock(): %s\n",
			        strerror(rc));
		}

		if(me->device->fresh) {
			rc = pthread_cond_signal(&me->cond);
			if (rc) {
				fprintf(stderr, "player_transmitter::query_loop(): pthread_cond_signal(): %s\n",
				        strerror(rc));
			}
			//fprintf(stderr, "pthread_cond_signal()\n");
		}

		if ((rc = pthread_mutex_unlock(&me->mtx))) {
			fprintf(stderr, "player_transmitter::query_loop(): pthread_mutex_lock(): %s\n",
			        strerror(rc));
		}

		pthread_testcancel();
	}

	return NULL;
}

// Set the robot speed
int player::position_set_cmd_vel(double vx, double vy, double va, int state)
{
	if (if_code != PLAYER_POSITION_CODE)
		return -1;

	player_position_cmd_t cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.xspeed = htonl((int) (vx * 1000.0));
	cmd.yspeed = htonl((int) (vy * 1000.0));
	cmd.yawspeed = htonl((int) (va * 180.0 / M_PI));
	cmd.state = state;
	cmd.type = 0;

	return playerc_client_write(client, device, &cmd, sizeof(cmd));
}

// Set the target pose
int player::position_set_cmd_pose(double gx, double gy, double ga, int state)
{
	if (if_code != PLAYER_POSITION_CODE)
		return -1;

	player_position_cmd_t cmd;

	memset(&cmd, 0, sizeof(cmd));

	cmd.xpos = htonl((int) (gx * 1000.0));
	cmd.ypos = htonl((int) (gy * 1000.0));
	cmd.yaw = htonl((int) (ga * 180.0 / M_PI));
	cmd.state = state;
	cmd.type = 1;

	return playerc_client_write(client, device, &cmd, sizeof(cmd));
}

// say phrase
int player::say(const char *str)
{
	if (if_code != PLAYER_SPEECH_CODE)
		return -1;

	player_speech_cmd_t cmd;

	memset(&cmd, 0, sizeof(cmd));

	if (str)
		strncpy ((char *) (cmd.string), str, PLAYER_SPEECH_MAX_STRING_LEN);

	return playerc_client_write(client, device, &cmd, sizeof(cmd));
}

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp
