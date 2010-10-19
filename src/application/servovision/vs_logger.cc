/*
 * vs_logger.cc
 *
 *  Created on: Jul 28, 2010
 *      Author: mboryn
 */

#include "vs_logger.h"

#include <stdexcept>

using namespace std;

namespace mrrocpp {

namespace ecp {

namespace servovision {

vs_logger::vs_logger(lib::configurator &config) :
	fp(NULL), config(config), counter(0)
{
	if (config.exists("reader_meassures_dir")) {
		reader_meassures_dir = config.value <string> ("reader_meassures_dir", lib::UI_SECTION);
	} else {
		reader_meassures_dir = config.return_default_reader_measures_path();
	}
}

vs_logger::~vs_logger()
{
}

void vs_logger::start()
{
	char filename[256];
	char file_date[128];

	time_t time_of_day = time(NULL);
	strftime(file_date, 40, "%g%m%d_%H-%M-%S", localtime(&time_of_day));

	sprintf(filename, "%s/%s_VS_readings", reader_meassures_dir.c_str(), file_date);
	fp = fopen(filename, "w");
	if(fp == NULL){
		throw logic_error(string("vs_logger::start(): error opening file ") +  filename);
	}

	fprintf(fp, "measure_time_sec;measure_time_nsec");
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 4; ++j)
			fprintf(fp, ";O_T_G_desired[%d][%d]", i, j);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 4; ++j)
			fprintf(fp, ";O_T_G_camera[%d][%d]", i, j);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 4; ++j)
			fprintf(fp, ";C_T_G[%d][%d]", i, j);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 4; ++j)
			fprintf(fp, ";error[%d][%d]", i, j);
	fprintf(fp, "\n");
}

void vs_logger::stop()
{
	if (fp != NULL) {
		fclose(fp);
		fp = NULL;
	}
}

void vs_logger::log(const lib::Homog_matrix &O_T_G_desired, const lib::Homog_matrix &O_T_G_camera, const lib::Homog_matrix &C_T_G, const lib::Homog_matrix &error)
{
	if (fp == NULL) {
		return;
	}

	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);

	fprintf(fp, "%ld;%ld", ts.tv_sec, ts.tv_nsec);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 4; ++j)
			fprintf(fp, ";%lg", O_T_G_desired(i, j));
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 4; ++j)
			fprintf(fp, ";%lg", O_T_G_camera(i, j));
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 4; ++j)
			fprintf(fp, ";%lg", C_T_G(i, j));
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 4; ++j)
			fprintf(fp, ";%lg", error(i, j));

	fprintf(fp, "\n");

	if (++counter >= MAX_ENTRIES) {
		fclose(fp);
		fp = NULL;
	}
}

} // namespace servovision

}

}
