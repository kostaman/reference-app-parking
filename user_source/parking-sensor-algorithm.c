// Copyright (c) Acconeer AB, 2018
// All rights reserved

#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "acc_log.h"
#include "acc_rss.h"
#include "acc_service.h"
#include "acc_service_envelope.h"
#include "acc_sweep_configuration.h"

#include "acc_version.h"

void handle_fatal_error(char *);


inline uint16_t min(uint16_t a, uint16_t b)
{
	return (a < b) ? a : b;
}


#define MAX_DATA_SIZE (3000)

/* default settings */

static const float DEFAULT_START_RANGE            = 0.12;
static const float DEFAULT_LENGTH_RANGE           = 0.48;
static const char  *DEFAULT_CALIBRATION_FILE_NAME = "parking.cal";
static const int   DEFAULT_SENSOR                 = 1;
static const int   NBR_OF_SWEEPS                  = 1;
static const int   FREQUENCY                      = 100;
static const int   DEFAULT_DELAY                  = 10;

#define  MAX_FILE_NAME_LENGTH (200)

typedef struct
{
	float        start_range;
	float        length_range;
	int          nbr_of_sweeps;
	int          frequency;
	acc_sensor_t sensor;
} radar_configuration_t;

typedef struct
{
	bool                  calibrate;
	bool                  read_calibration_file;
	radar_configuration_t radar_config;
	int                   loglevel;
	char                  calibration_file_name[MAX_FILE_NAME_LENGTH + 1];
	int                   time_delay;
	bool                  delay;
} app_configuration_t;

typedef struct datapoint
{
	float dist;
	float amp;
} Datapoint;


/**
 * @brief Initialize configuration struct with default values
 *
 * @param[out] app_config
 */


void init_configuration(app_configuration_t *app_config)
{
	app_config->calibrate                  = false;
	app_config->read_calibration_file      = false;
	app_config->radar_config.start_range   = DEFAULT_START_RANGE;
	app_config->radar_config.length_range  = DEFAULT_LENGTH_RANGE;
	app_config->radar_config.nbr_of_sweeps = NBR_OF_SWEEPS;
	app_config->radar_config.frequency     = FREQUENCY;
	app_config->radar_config.sensor        = DEFAULT_SENSOR;
	app_config->loglevel                   = ACC_LOG_LEVEL_ERROR;
	app_config->time_delay                 = DEFAULT_DELAY;
	app_config->delay                      = false;
	strcpy(app_config->calibration_file_name, DEFAULT_CALIBRATION_FILE_NAME);
}


/**
 * @brief Print usage information to stdout
 *
 * @param[in] program_name
 */


static void print_usage(const char *program_name)
{
	fprintf(stderr, "Usage: %s [OPTIONS]\n", program_name);
	fprintf(stderr, "\n");
	fprintf(stderr, "-h, --help                    this help\n");
	fprintf(stderr, "-s, --sensor                  sensor to use, default %u\n", DEFAULT_SENSOR);
	fprintf(stderr, "-c, --calibrate               record read empty parking spot data and store calibration file\n");
	fprintf(stderr, "-f, --calibration-file        name of the calibration file, default %s\n", DEFAULT_CALIBRATION_FILE_NAME);
	fprintf(stderr, "-a, --range-start             start measure at this distance [m], default %.3f\n", (double)DEFAULT_START_RANGE);
	fprintf(stderr, "-d, --delay                   do multiple measurements with a time delay in between (time in seconds)\n");
	fprintf(stderr, "-v, --verbose                 enable verbose logging\n");
}


/**
 * @brief Parse command line options and update configuration struct
 *
 * @param[in]  argc Number of arguments passed to the main function
 * @param[in]  argv Array with arguments passed to the main function
 * @param[out] app_config configuration data to be updated
 */


static acc_status_t parse_options(int argc, char *argv[], app_configuration_t *app_config)
{
	static struct option long_options[] =
	{
		{"help",                    no_argument,          0,    'h'},
		{"sensor",                  required_argument,    0,    's'},
		{"calibrate",               no_argument,          0,    'c'},
		{"calibration-file",        required_argument,    0,    'f'},
		{"range-start",             required_argument,    0,    'a'},
		{"delay",                   required_argument,    0,    'd'},
		{"verbose",                 no_argument,          0,    'v'},
		{NULL,                      0,                    NULL,   0}
	};

	int character_code;
	int option_index = 0;

	init_configuration(app_config);

	while ((character_code = getopt_long(argc, argv, "s:a:f:d:cvh?:", long_options, &option_index)) != -1)
	{
		switch (character_code)
		{
			case 's':
			{
				app_config->radar_config.sensor = atoi(optarg);
				break;
			}

			case 'c':
			{
				app_config->calibrate = true;
				break;
			}

			case 'f':
			{
				app_config->read_calibration_file = true;
				strncpy(app_config->calibration_file_name, optarg, MAX_FILE_NAME_LENGTH);
				app_config->calibration_file_name[MAX_FILE_NAME_LENGTH] = '\0';
				break;
			}

			case 'a':
			{
				char *next;
				app_config->radar_config.start_range = strtof(optarg, &next);
				break;
			}

			case 'd':
			{
				app_config->delay      = true;
				app_config->time_delay = atoi(optarg);
				break;
			}

			case 'h':
			case '?':
			{
				print_usage(argv[0]);
				exit(0);
			}

			case 'v':
			{
				app_config->loglevel = ACC_LOG_LEVEL_INFO;
				break;
			}
		}
	}

	return ACC_STATUS_SUCCESS;
}


/**
 * @brief Decides if car is present based on average amplitude, peak amplitudes, and calibration data.
 * This algorithm needs calibration.
 *
 * @param[in] avg_peak_amp The max peak amplitude from the collected envelope data
 * @param[in] avg_calib_amp The threshold from calibration which decides whether the algorithm should output 1 or 0
 * @param[in] avg_amp_factor Amplitude factor
 * @return -1 if no car, and 1 if car is present.
 **/
int car_present(float avg_peak_amp, float avg_calib_amp, float avg_amp_factor)
{
	if (avg_peak_amp <= avg_calib_amp * avg_amp_factor * 4)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


/**
 * @brief Organizes collected amplitude data 'amp' with its distance 'dist'. Calculates the
 * start-to-end range and adds each amplitude 'amp' to a distance 'dist' with
 * 'step' interval, within the range.
 *
 * @param[out] data Array of the collected envelope data organized as datapoints with amplitude and distance
 * @param[in]  length Length of datapoint array
 * @param[in]  start Corresponds to start range in the application configuration
 * @param[in]  end The end range for the distance the sensor is measuring
 **/
void format_data(Datapoint *data, uint16_t *amp, int length, float start, float end)
{
	float range = end - start;
	float step  = range/length;

	for (int i = 0; i < length; i++)
	{
		data[i].dist = start + step*i;
		data[i].amp  = amp[i];
	}
}


/**
 * @brief Calculates the average amplitude for given data.
 *
 * @param[in] data Array of envelope data
 * @param[in] length The length of the envelope data array
 * @return a float number corresponding to the average amplitude for all sweeps
 **/
float get_average_amplitude(Datapoint *data, int length)
{
	float sum = 0;

	for (int i = 0; i < length; i++)
	{
		sum += data[i].amp;
	}

	return sum/length;
}


/**
 * @brief Calculates the max peak for given data. Assumes there is only one
 * sweep in data.
 *
 * @param[in] data Array of envelope data
 * @param[in] length The length of the envelope data array
 * @return a datapoint with average max peak amplitude and distance from all sweeps.
 **/
Datapoint get_max_peak(Datapoint *data, int length)
{
	Datapoint max;

	max.amp  = -1;
	max.dist = -1;

	for (int i = 0; i < length; i++)
	{
		if (data[i].amp > max.amp)
		{
			max = data[i];
		}
	}

	return max;
}


/**
 * @brief Calculate threashold from calibration data stored in file
 *
 * The threshold_data[i] is set to  captured envelope data from calibration.
 *
 * @param[in]  app_config configuration data
 * @param[out] avg_calib_amp The average amplitude value from the threshold data
 * @param[out] peak_amp The datapoint with highest amplitude and its corresponding distance
 * @param[out] avg_amp_factor = peak_amp/avg_calib_amp
 */
void read_and_calculate_threshold(app_configuration_t *app_config, float *avg_calib_amp, Datapoint *peak_amp, float *avg_amp_factor)
{
	uint16_t threshold_data[MAX_DATA_SIZE];
	float    start;
	float    length;
	unsigned n;
	uint16_t res;
	FILE     *fin;

	fin = fopen(app_config->calibration_file_name, "r");

	if (fin == NULL)
	{
		handle_fatal_error("Unable to read calibration data file");
	}

	res  =  fscanf(fin, "start %f\n", &start);
	res += fscanf(fin, "length %f\n", &length);
	res += fscanf(fin, "n %u\n", &n);

	if (res != 3)
	{
		handle_fatal_error("Calibration data file format error.\n");
	}

	res = 0;
	for (unsigned int i = 0; i < n; i++)
	{
		res += fscanf(fin, "%hu", &threshold_data[i]);
	}

	if (res != n)
	{
		handle_fatal_error("Calibration data file format error.\n");
	}

	if (start != app_config->radar_config.start_range)
	{
		printf("Setting start_range to %1.2f due to calibration file\n", (double)start);
		app_config->radar_config.start_range = start;
	}

	if (length < app_config->radar_config.length_range)
	{
		printf("Setting length_range to %1.2f due to calibration file\n", (double)length);
		app_config->radar_config.length_range = length;
	}

	n = min(n, MAX_DATA_SIZE);
	memset(threshold_data, 0, n);

	Datapoint th_data[n];
	format_data(th_data, threshold_data, n, app_config->radar_config.start_range,
	            app_config->radar_config.start_range + app_config->radar_config.length_range);

	*avg_calib_amp  = get_average_amplitude(th_data, n);
	*peak_amp       = get_max_peak(th_data, n);
	*avg_amp_factor = peak_amp->amp / *avg_calib_amp;
}


/**
 * @brief Handle fatal errors by printing error message and terminating the program
 *
 * @param[in]  message error message
 */


void handle_fatal_error(char *message)
{
	fprintf(stderr, "Fatal error: %s\n", message);
	exit(EXIT_FAILURE);
}


/**
 * @brief Capture envelope data and write to calibration file
 *
 * @param[in]   app_config Configuration data
 * @param[in]   envelope_configuration The envelope configuration
 * @returns     An envelope service instance
 */
acc_service_handle_t create_sensor_service(app_configuration_t *app_config, acc_service_configuration_t envelope_configuration)
{
	//set service profile
	acc_service_envelope_profile_set(envelope_configuration, ACC_SERVICE_ENVELOPE_PROFILE_MAXIMIZE_SNR);

	//create sweep configuration
	acc_sweep_configuration_t sweep_configuration = acc_service_get_sweep_configuration(envelope_configuration);
	if (sweep_configuration == NULL)
	{
		handle_fatal_error("Sweep configuration not available");
	}

	//set sweep configs
	acc_sweep_configuration_requested_range_set(sweep_configuration, app_config->radar_config.start_range, app_config->radar_config.length_range);
	acc_sweep_configuration_repetition_mode_streaming_set(sweep_configuration, 100);
	acc_sweep_configuration_sensor_set(sweep_configuration, app_config->radar_config.sensor);

	//create service
	acc_service_handle_t envelope_handle = acc_service_create(envelope_configuration);
	if (envelope_handle == NULL)
	{
		handle_fatal_error("acc_service_create() failed.");
	}

	return envelope_handle;
}


/**
 * @brief Captures one sweep of envelope data
 *
 * @param[in]   envelope_handle The envelope service instance
 * @param[out]  envelope_data Array with envelope data
 * @param[in]   data_length Max length of envelope data array
 * @returns     Actual length of the envelope_data array
 */
uint16_t get_one_sweep(acc_service_handle_t envelope_handle, uint16_t *envelope_data, uint16_t data_length)
{
	//get number of samples (data length) that will be used
	acc_service_envelope_metadata_t envelope_metadata;

	acc_service_envelope_get_metadata(envelope_handle, &envelope_metadata);
	uint16_t actual_data_length = min(envelope_metadata.data_length, data_length);

	//start doing measurements
	acc_service_status_t service_status = acc_service_activate(envelope_handle);

	//read envelope data from sensor
	acc_service_envelope_result_info_t result_info;

	service_status = acc_service_envelope_get_next(envelope_handle,
	                                               envelope_data,
	                                               envelope_metadata.data_length,
	                                               &result_info);
	if (service_status != ACC_SERVICE_STATUS_OK)
	{
		handle_fatal_error("acc_service_envelope_get_next() failed.");
	}

	return actual_data_length;
}


/**
 * @brief Deactivate and destroy envelope service instance
 *
 * @param[in]   envelope_handle The envelope service instance
 */
void close_sensor_service(acc_service_handle_t envelope_handle)
{
	acc_service_deactivate(envelope_handle);
	acc_service_destroy(&envelope_handle);
}


/**
 * @brief Capture envelope data and write to calibration file
 *
 * @param[in]   app_config Configuration data
 * @param[in]   envelope_handle The envelope service instance
 */
void write_calibration_data(app_configuration_t *app_config, acc_service_configuration_t envelope_configuration)
{
	uint16_t data_len = MAX_DATA_SIZE;
	uint16_t data[data_len];

	FILE *fout;

	fout = fopen(app_config->calibration_file_name, "w");

	acc_service_handle_t envelope_handle = create_sensor_service(app_config, envelope_configuration);

	data_len = get_one_sweep(envelope_handle, data, data_len);

	if (fout == NULL)
	{
		handle_fatal_error("Unable to write calibration data to file\n");
	}

	fprintf(fout, "start %f\n", (double)app_config->radar_config.start_range);
	fprintf(fout, "length %f\n", (double)app_config->radar_config.length_range);
	fprintf(fout, "n %u\n", data_len);

	for (int i = 0; i < data_len; i++)
	{
		fprintf(fout, "%d ", data[i]);
	}

	fclose(fout);
	close_sensor_service(envelope_handle);
}


/**
 * @brief Get a detection (car/empty) from the envelope data
 *
 * Uses algorithm car_present() or car_present2() depending on input arguments in main()
 *
 * @param[in]   app_config Configuration data
 * @param[in]   envelope_handle The envelope service instance
 * @param[out]  avg_calib_amp The average amplitude value from the threshold data
 * @param[out]  avg_amp_factor = peak_amp/avg_calib_amp
 * @returns     1 if there is a car, 0 if the parking spot is empty
 */
int get_detection(app_configuration_t *app_config, acc_service_configuration_t envelope_configuration, float *avg_calib_amp, float *avg_amp_factor)
{
	uint16_t data_len = MAX_DATA_SIZE;
	uint16_t envelope_data[data_len];

	acc_service_handle_t envelope_handle = create_sensor_service(app_config, envelope_configuration);

	data_len = get_one_sweep(envelope_handle, envelope_data, MAX_DATA_SIZE);
	Datapoint data[data_len];
	format_data(data, envelope_data, data_len, app_config->radar_config.start_range,
	            app_config->radar_config.start_range + app_config->radar_config.length_range);

	Datapoint avg_peak = get_max_peak(data, data_len);

	int result = -2;
	result = car_present(avg_peak.amp, *avg_calib_amp, *avg_amp_factor);
	printf("%d\n", result);
	if (app_config->delay)
	{
		int first_res = -2;

		while (first_res != result)
		{
			first_res = result;
			sleep(app_config->time_delay);

			data_len = get_one_sweep(envelope_handle, envelope_data, data_len);
			format_data(data, envelope_data, data_len, app_config->radar_config.start_range,
			            app_config->radar_config.start_range + app_config->radar_config.length_range);

			avg_peak = get_max_peak(data, data_len);

			result = car_present(avg_peak.amp, *avg_calib_amp, *avg_amp_factor);
			printf("%d\n", result);
		}
	}

	close_sensor_service(envelope_handle);
	return result;
}


int main(int argc, char *argv[])
{
	float     avg_calib_amp = 0;
	Datapoint peak_amp;

	peak_amp.amp  = 0;
	peak_amp.dist = 0;
	float avg_amp_factor = 0;

	app_configuration_t app_config;
	parse_options(argc, argv, &app_config);
	acc_log_set_level(app_config.loglevel, NULL);
	printf("start ref_app\n");

	//activate radar system services
	if (!acc_rss_activate())
	{
		handle_fatal_error("acc_rss_activate() failed");
	}

	printf("rss_activated\n");

	//create envelope configuration
	acc_service_configuration_t envelope_configuration = acc_service_envelope_configuration_create();
	if (envelope_configuration == NULL)
	{
		handle_fatal_error("acc_service_envelope_configuration_create() failed.");
	}

	if (app_config.calibrate)
	{
		write_calibration_data(&app_config, envelope_configuration);
		printf("Calibration done. Saved in file %s\n", app_config.calibration_file_name);

		return EXIT_SUCCESS;
	}

	if (app_config.read_calibration_file)
	{
		read_and_calculate_threshold(&app_config, &avg_calib_amp, &peak_amp, &avg_amp_factor);
	}
	else
	{
		printf("Please specify calibration file.\n");
		print_usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	printf("Start range: %f\n", (double)app_config.radar_config.start_range);

	int result = get_detection(&app_config, envelope_configuration, &avg_calib_amp, &avg_amp_factor);

	acc_rss_deactivate();

	//print results
	if (result == 1)
	{
		printf("\nCar detected.\n");
	}
	else
	{
		printf("\nNothing detected.\n");
	}

	return EXIT_SUCCESS;
}
