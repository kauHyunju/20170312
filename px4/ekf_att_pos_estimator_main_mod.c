/* Modify to C version from C++ version */


/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ekf_att_pos_estimator_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

/*
 * @file ekf_att_pos_estimator_main_mod.c
 * @author Minwook Jang <mwjang86@gmail.com>
 */

//#include "AttitudePositionEstimatorEKF.h"
//#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>


//#include "AttitudePositionEstimatorEKF_mod.h"
#include "abstract.h"
//#include "estimator_22states_mod.h"
//#include "drv_mod.h"
/*
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
//#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>
 */
#define MAVLINK_LOG_DEVICE          "/dev/mavlink"
#define distance_sensor "distance_sensor"
#define sensor_baro	"sensor_baro"
#define airspeed	"airspeed"	
#define vehicle_gps_position	"vehicle_gps_position"
#define vehicle_status	"vehicle_status"
#define parameter_update	"parameter_update"
#define home_position	"home_position"
#define vehicle_land_detected	"vehicle_land_detected"
#define actuator_armed	"actuator_armed"
#define sensor_combined	"sensor_combined"
#define vehicle_attitude	"vehicle_attitude"
#define vehicle_local_position	"vehicle_local_position"
#define vehicle_global_position	"vehicle_global_position"
#define wind_estimate	"wind_estimate"
#define estimator_status	"estimator_status"
//char* estimator_status = "estimator_status";

#define SCHED_DEFAULT	1		//SCHED_FIFO = 1, SCHED_RR = 2 로 정의되어있음
//config 파일 분석결과 SCHED_DEFAULT = 1 임
#define SCHED_PRIORITY_MAX	100
typedef unsigned int size_t;
typedef unsigned long long uint64_t;
typedef _Bool bool;
typedef uint64_t hrt_abstime;

static uint64_t IMUmsec = 0;
//uint64_t IMUmsec = 0;
static uint64_t IMUusec = 0;
//uint64_t IMUusec = 0;

//Constants
float rc = 10.0f;	// RC time constant of 1st order LPF in seconds
uint64_t FILTER_INIT_DELAY = 1 * 1000 * 1000;	///< units: microseconds
float POS_RESET_THRESHOLD = 5.0f;				///< Seconds before we signal a total GPS failure
unsigned MAG_SWITCH_HYSTERESIS = 10;	///< Ignore the first few mag failures (which amounts to a few milliseconds)
unsigned GYRO_SWITCH_HYSTERESIS = 5;	///< Ignore the first few gyro failures (which amounts to a few milliseconds)
unsigned ACCEL_SWITCH_HYSTERESIS = 5;	///< Ignore the first few accel failures (which amounts to a few milliseconds)

/**
 * estimator app start / stop handling function
 *
 * @ingroup apps
 */

/*
   uint32_t millis()
   {
   return IMUmsec;
   }

   uint64_t getMicros()
   {
   return IMUusec;
   }
 */

//struct AttPosEKF *_ekf;

struct accel_report {
	int timestamp;
	int error_count;
	float x;        /**< acceleration in the NED X board axis in m/s^2 */
	float y;        /**< acceleration in the NED Y board axis in m/s^2 */
	float z;        /**< acceleration in the NED Z board axis in m/s^2 */
	float temperature;  /**< temperature in degrees celsius */
	float range_m_s2;   /**< range in m/s^2 (+- this value) */
	float scaling;

	int x_raw;
	int y_raw;
	int z_raw;
	int temperature_raw;
};

/** accel scaling factors; Vout = Vscale * (Vin + Voffset) */
struct accel_scale {
	float   x_offset;
	float   x_scale;
	float   y_offset;
	float   y_scale;
	float   z_offset;
	float   z_scale;
};

//drv_baro.h

struct baro_report {
	float pressure;
	float altitude;
	float temperature;
	int timestamp;
	int error_count;
};



//drv_gyro.h
#define GYROIOCGSCALE 1
#define GYRO_BASE_DEVICE_PATH   "/dev/gyro"
struct gyro_report {
	int timestamp;
	int error_count;
	float x;        /**< angular velocity in the NED X board axis in rad/s */
	float y;        /**< angular velocity in the NED Y board axis in rad/s */
	float z;        /**< angular velocity in the NED Z board axis in rad/s */
	float temperature;  /**< temperature in degrees celcius */
	float range_rad_s;
	float scaling;

	int x_raw;
	int y_raw;
	int z_raw;
	int temperature_raw;
};

/** gyro scaling factors; Vout = (Vin * Vscale) + Voffset */
struct gyro_scale {
	float   x_offset;
	float   x_scale;
	float   y_offset;
	float   y_scale;
	float   z_offset;
	float   z_scale;
};


//drv_mag.h


#define MAG_BASE_DEVICE_PATH    "/dev/mag"
#define MAGIOCGSCALE 3

struct mag_report {
	int timestamp;
	int error_count;
	float x;
	float y;
	float z;
	float range_ga;
	float scaling;
	float temperature;

	int x_raw;
	int y_raw;
	int z_raw;
};

/** mag scaling factors; Vout = (Vin * Vscale) + Voffset */
struct mag_scale {
	float   x_offset;
	float   x_scale;
	float   y_offset;
	float   y_scale;
	float   z_offset;
	float   z_scale;
};
struct vehicle_attitude_s
{
	float q[22];
	_Bool q_valid;
	_Bool R_valid;
	float timestamp; 
	int roll;
	int pitch;
	int yaw;

	float rollspeed;
	float pitchspeed;
	float yawspeed;

	float rate_offsets[3];

};


struct airspeed_s
{
	float   true_airspeed_unfiltered_m_s;
	float   true_airspeed_m_s;
};

struct vehicle_status_s
{
	int HIL_STATE_ON;
	int hil_state;
	_Bool is_rotary_wing;
};



struct vehicle_global_position_s
{
	int timestamp;
	int time_utc_usec;

	float lat;
	float lon;
	float alt;

	double yaw;
	double eph;
	double epv;

	_Bool v_xy_valid;
	_Bool v_z_valid;

	float vel_n;
	float vel_e;
	float vel_d;
	_Bool terrain_alt_valid;
	float terrain_alt;

	_Bool dead_reckoning;
};

struct vehicle_local_position_s
{
	int timestamp;
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
	float yaw;
	float alt;
	_Bool xy_valid;
	_Bool z_valid;
	_Bool v_xy_valid;
	_Bool v_z_valid;
	_Bool xy_global;
	_Bool z_global;
	int ref_timestamp;
	float ref_lat;
	float ref_lon;
	float ref_alt;
	float dist_bottom;
	float dist_bottom_rate;
	int surface_bottom_timestamp;
	_Bool dist_bottom_valid;
	float eph;
	float epv;

};

struct vehicle_gps_position_s
{

	_Bool vel_ned_valid;
	int timestamp_position;

	double time_utc_usec;
	int fix_type;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	int lat;
	int lon;
	float eph;
	float epv;
	float alt;

};

struct wind_estimate_s
{
	int timestamp;
	float windspeed_north;
	float windspeed_east;
	float covariance_north;
	float covariance_east;



};


struct distance_sensor_s
{

	int timestamp;
	float current_distance;
	float min_distance;
	float max_distance;

};
struct vehicle_land_detected_s
{
	_Bool landed;
};
struct actuator_armed_s
{

	float  armed;

};

struct sensor_combined_s
{
	float gyro_rad_s[3];
	float gyro_rad1_s[3];
	float gyro1_rad_s[3];

	float accelerometer_m_s2[3];
	float accelerometer1_m_s2[3];

	float magnetometer_ga[3];
	int gyro_errcount;
	int gyro1_errcount;
	int gyro2_errcount;
	int accelerometer_errcount;
	int accelerometer1_errcount;
	int accelerometer2_errcount;
	int magnetometer_errcount;
	int magnetometer1_errcount;
	int magnetometer2_errcount;

	int timestamp;
	int accelerometer_timestamp;
	int magnetometer_timestamp;
	int magnetometer1_timestamp;

};

struct map_projection_reference_s {
	double lat_rad;
	double lon_rad;
	double sin_lat;
	double cos_lat;
	_Bool init_done;
	int timestamp;
};

struct LowPassFilter2p
{

	/**
	 *  constructor
	 */
	void (*constructor)(struct LowPassFilter2p *, float sample_freq, float cutoff_freq);

	/**
	 * Change filter parameters
	 */
	void (*set_cutoff_frequency)(struct LowPassFilter2p *,float sample_freq, float cutoff_freq);

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	float (*apply)(struct LowPassFilter2p *,float sample);
	/**
	 * Return the cutoff frequency
	 */

	float (*get_cutoff_freq)(struct LowPassFilter2p *);
	/**
	 * Reset the filter state to this value
	 */
	float (*reset)(struct LowPassFilter2p * ,float sample);

	float           _cutoff_freq;
	float           _a1;
	float           _a2;
	float           _b0;
	float           _b1;
	float           _b2;
	float           _delay_element_1;        // buffered sample -1
	float           _delay_element_2;        // buffered sample -2
};

struct AttitudePositionEstimatorEKF
{
	/**
	 * Constructor
	 */
	void (*constructor) (struct AttitudePositionEstimatorEKF *);


	/* we do not want people ever copying this class */
	//AttitudePositionEstimatorEKF(const AttitudePositionEstimatorEKF& that) = delete;
	//AttitudePositionEstimatorEKF operator=(const AttitudePositionEstimatorEKF&) = delete;

	/**
	 * Destructor, also kills the sensors task.
	 */
	void (*destructor) (struct AttitudePositionEstimatorEKF *);

	/**
	 * Start the sensors task.
	 *
	 * @return  OK on success.
	 */
	int (*start)(struct AttitudePositionEstimatorEKF *);

	/**
	 * Task status
	 *
	 * @return  true if the mainloop is running
	 */
	bool (*task_running)(struct AttitudePositionEstimatorEKF *);


	/**
	 * Print the current status.
	 */
	void (*print_status) (struct AttitudePositionEstimatorEKF *);



	/**
	 * Trip the filter by feeding it NaN values.
	 */
	int (*trip_nan) (struct AttitudePositionEstimatorEKF *);


	/**
	 * Enable logging.
	 *
	 * @param   enable Set to true to enable logging, false to disable
	 */
	int (*enable_logging) (struct AttitudePositionEstimatorEKF *,bool enable);


	/**
	 * Set debug level.
	 *
	 * @param   debug Desired debug level - 0 to disable.
	 */
	int (*set_debuglevel) (struct AttitudePositionEstimatorEKF *,unsigned debug);

	bool        _task_should_exit;      /**< if true, sensor task should exit */
	bool        _task_running;          /**< if true, task is running in its mainloop */
	int     _estimator_task;        /**< task handle for sensor task */

	struct     sensor_combined_s            _sensor_combined;

	struct map_projection_reference_s   _pos_ref;

	float                       _filter_ref_offset;   /**< offset between initial baro reference and GPS init baro altitude */
	float                       _baro_gps_offset;   /**< offset between baro altitude (at GPS init time) and GPS altitude */
	hrt_abstime                 _last_debug_print;

	//perf_counter_t  _loop_perf;         ///< loop performance counter
	//struct perf_ctr_header _loop_perf;
	//perf_counter_t  _loop_intvl;        ///< loop rate counter
	//perf_counter_t  _perf_gyro;         ///<local performance counter for gyro updates
	//perf_counter_t  _perf_mag;          ///<local performance counter for mag updates
	//perf_counter_t  _perf_gps;          ///<local performance counter for gps updates
	//perf_counter_t  _perf_baro;         ///<local performance counter for baro updates
	//perf_counter_t  _perf_airspeed;     ///<local performance counter for airspeed updates
	//perf_counter_t  _perf_reset;        ///<local performance counter for filter resets

	float           _gps_alt_filt;
	float           _baro_alt_filt;
	float           _covariancePredictionDt;  ///< time lapsed since last covariance prediction
	bool            _gpsIsGood;               ///< True if the current GPS fix is good enough for us to use
	uint64_t        _previousGPSTimestamp;    ///< Timestamp of last good GPS fix we have received
	bool            _baro_init;
	bool            _gps_initialized;
	hrt_abstime     _filter_start_time;
	hrt_abstime     _last_sensor_timestamp;
	hrt_abstime     _last_run;
	hrt_abstime     _distance_last_valid;
	bool            _gyro_valid;
	bool            _accel_valid;
	bool            _mag_valid;
	int             _gyro_main;         ///< index of the main gyroscope
	int             _accel_main;        ///< index of the main accelerometer
	int             _mag_main;          ///< index of the main magnetometer
	bool            _ekf_logging;       ///< log EKF state
	//unsigned        _debug;             ///< debug level - default 0


	//hj
	struct			vehicle_attitude_s			_att;
	struct			vehicle_local_position_s	_local_pos;
	struct			vehicle_global_position_s	_global_pos;
	struct			wind_estimate_s				_wind;
	//hj
	bool            _newHgtData;
	bool            _newAdsData;
	bool            _newDataMag;
	bool            _newRangeData;

	int             _mavlink_fd;

	struct {
		/*int32_t vel_delay_ms;
		  int32_t pos_delay_ms;
		  int32_t height_delay_ms;
		  int32_t mag_delay_ms;
		  int32_t tas_delay_ms;
		 */
		float vel_delay_ms;
		float pos_delay_ms;
		float height_delay_ms;
		float mag_delay_ms;
		float tas_delay_ms;
		float velne_noise;
		float veld_noise;
		float posne_noise;
		float posd_noise;
		float mag_noise;
		float gyro_pnoise;
		float acc_pnoise;
		float gbias_pnoise;
		float abias_pnoise;
		float mage_pnoise;
		float magb_pnoise;
		float eas_noise;
		float pos_stddev_threshold;
	}       _parameters;            /**< local copies of interesting parameters */

	struct {
		int* vel_delay_ms;
		int* pos_delay_ms;
		int* height_delay_ms;
		int* mag_delay_ms;
		int* tas_delay_ms;
		int* velne_noise;
		int* veld_noise;
		int* posne_noise;
		int* posd_noise;
		int* mag_noise;
		int* gyro_pnoise;
		int* acc_pnoise;
		int* gbias_pnoise;
		int* abias_pnoise;
		int* mage_pnoise;
		int* magb_pnoise;
		int* eas_noise;
		int* pos_stddev_threshold;
	}       _parameter_handles;     /**< handles for interesting parameters */


	/* Low pass filter for attitude rates */
	//constructor
	//math::LowPassFilter2p _LP_att_P;
	//math::LowPassFilter2p _LP_att_Q;
	//math::LowPassFilter2p _LP_att_R;
	struct LowPassFilter2p _LP_att_P;
	struct LowPassFilter2p _LP_att_Q;
	struct LowPassFilter2p _LP_att_R;


	/**
	 * Update our local parameter cache.
	 */
	int (*parameters_update) (struct AttitudePositionEstimatorEKF *);

	/**
	 * Update control outputs
	 *
	 */
	void (*control_update) (struct AttitudePositionEstimatorEKF *);


	/**
	 * Check for changes in vehicle status.
	 */
	void (*vehicle_status_poll) (struct AttitudePositionEstimatorEKF *);


	/**
	 * Shim for calling task_main from task_create.
	 */
	void (*task_main_trampoline) (struct AttitudePositionEstimatorEKF *,int argc,char *argv[]);

	/**
	 * Main filter task.
	 */
	void (*task_main) (struct AttitudePositionEstimatorEKF *);


	/**
	 * Check filter sanity state
	 *
	 * @return zero if ok, non-zero for a filter error condition.
	 */

	int (*check_filter_state) (struct AttitudePositionEstimatorEKF *);

	/**
	 * @brief
	 *   Publish the euler and quaternions for attitude estimation
	 **/
	void (*publishAttitude) (struct AttitudePositionEstimatorEKF *);


	/**
	 * @brief
	 *   Publish local position relative to reference point where filter was initialized
	 **/
	void (*publishLocalPosition) (struct AttitudePositionEstimatorEKF *);


	/**
	 * @brief
	 *   Publish global position estimation (WSG84 and AMSL).
	 *   A global position estimate is only valid if we have a good GPS fix
	 **/
	void (*publishGlobalPosition)(struct AttitudePositionEstimatorEKF *);


	/**
	 * @brief
	 *   Publish wind estimates for north and east in m/s
	 **/
	void (*publishWindEstimate) (struct AttitudePositionEstimatorEKF *);


	/**
	 * @brief
	 *   Runs the sensor fusion step of the filter. The parameters determine which of the sensors
	 *   are fused with each other
	 **/
	void (*updateSensorFusion) (struct AttitudePositionEstimatorEKF *,const bool fuseGPS, const bool fuseMag,const bool fuseRangeSensor, const bool fuseBaro, const bool fuseAirSpeed);


	/**
	 * @brief
	 *   Initialize first time good GPS fix so we can get a reference point to calculate local position from
	 *   Should only be required to call once
	 **/
	void (*initializeGPS) (struct AttitudePositionEstimatorEKF *);


	/**
	 * Initialize the reference position for the local coordinate frame
	 */
	void (*initReferencePosition) (struct AttitudePositionEstimatorEKF *,hrt_abstime timestamp,double lat, double lon, float gps_alt, float baro_alt);


	/**
	 * @brief
	 *   Polls all uORB subscriptions if any new sensor data has been publish and sets the appropriate
	 *   flags to true (e.g newDataGps)
	 **/
	void (*pollData) (struct AttitudePositionEstimatorEKF *);
};

const int ERROR=-1;

struct AttitudePositionEstimatorEKF *_aekf;
struct LowPassFilter2p *_lpf;
struct AttPosEKF *_ekf;
struct Vector3f *_vec3f;

struct estimator_status_s{

	    int nan_flags;
		int health_flags;
		int timeout_flags;
		float states[32];
		int n_states;
		float covariances[28];
		float timestamp;
};


void usleep(int usec){
	int sleep_usec;
	sleep_usec=usec;
}

int orb_publish(char * meta, int st_size){

	char* topic_name;
	topic_name=meta;
	int topic_size;
	topic_size=st_size;

	return 1;
}



//constructor
void LowPassFilter2p_constructor(struct LowPassFilter2p * _lpf,float sample_freq, float cutoff_freq)
{

	/*_lpf->set_cutoff_frequency = LowPassFilter2p_set_cutoff_frequency;
	  _lpf->apply = LowPassFilter2p_apply;
	  _lpf->get_cutoff_freq = LowPassFilter2p_get_cutoff_freq;
	  _lpf->reset = LowPassFilter2p_reset;

	  _lpf->_cutoff_freq = cutoff_freq,
	  _lpf->_a1 = 0.0f,
	  _lpf->_a2 = 0.0f,
	  _lpf->_b0 = 0.0f,
	  _lpf->_b1 = 0.0f,
	  _lpf->_b2 = 0.0f,
	  _lpf->_delay_element_1 = 0.0f,
	  _lpf->_delay_element_2= 0.0f;
	// set initial parameters
	_lpf->set_cutoff_frequency(_lpf,sample_freq, cutoff_freq);*/
}
void LowPassFilter2p_get_cutoff_freq(struct LowPassFilter2p * get_cutoff_freq)
{
	//return _lpf->_cutoff_freq;
}

void LowPassFilter2p_set_cutoff_frequency(struct LowPassFilter2p * _lpf, float sample_freq, float cutoff_freq)
{
	/*_lpf->_cutoff_freq = cutoff_freq;
	  if (_lpf->_cutoff_freq <= 0.0f) {
	// no filtering
	return;
	}
	float fr = sample_freq/_lpf->_cutoff_freq;
	float ohm = _tanf(M_PI_F/fr);
	float c = 1.0f+2.0f*_cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
	_lpf->_b0 = ohm*ohm/c;
	_lpf->_b1 = 2.0f*_lpf->_b0;
	_lpf->_b2 = _lpf->_b0;
	_lpf->_a1 = 2.0f*(ohm*ohm-1.0f)/c;
	_lpf->_a2 = (1.0f-2.0f*_cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
	 */
}

float LowPassFilter2p_apply(struct LowPassFilter2p * _lpf, float sample)
{
	/*	if (_lpf->_cutoff_freq <= 0.0f) {
	// no filtering
	return sample;
	}

	// do the filtering
	float delay_element_0 = sample - _lpf->_delay_element_1 * _lpf->_a1 - _lpf->_delay_element_2 * _lpf->_a2;
	if (isnan(delay_element_0) || isinf(delay_element_0)) {
	// don't allow bad values to propagate via the filter
	delay_element_0 = sample;
	}*/
	float output;
	/*	= delay_element_0 * _lpf->_b0 + _lpf->_delay_element_1 * _lpf->_b1 + _lpf->_delay_element_2 * _lpf->_b2;

		_lpf->_delay_element_2 = _lpf->_delay_element_1;
		_lpf->_delay_element_1 = delay_element_0;
	 */
	// return the value.  Should be no need to check limits
	return output;
}

void LowPassFilter2p_reset(struct LowPassFilter2p * _lpf,float sample) {
	/*	float dval = sample / (_lpf->_b0 + _lpf->_b1 + _lpf->_b2);
		_lpf->_delay_element_1 = dval;
		_lpf->_delay_element_2 = dval;
		return _lpf->apply(_lpf,sample);*/
}


struct AttitudePositionEstimatorEKF	*g_estimator;// = NULL;
void AttitudePositionEstimatorEKF_constructor(struct AttitudePositionEstimatorEKF * _aekf)
{

	/*	_aekf->start = AttitudePositionEstimatorEKF_start;
		_aekf->task_running = AttitudePositionEstimatorEKF_task_running;
		_aekf->print_status = AttitudePositionEstimatorEKF_print_status;
		_aekf->trip_nan = AttitudePositionEstimatorEKF_trip_nan;
		_aekf->enable_logging = AttitudePositionEstimatorEKF_enable_logging;
		_aekf->set_debuglevel = AttitudePositionEstimatorEKF_set_debug_level;
		_aekf->parameters_update = AttitudePositionEstimatorEKF_parameters_update;
		_aekf->control_update = AttitudePositionEstimatorEKF_control_update;
		_aekf->vehicle_status_poll = AttitudePositionEstimatorEKF_vehicle_status_poll;
		_aekf->task_main_trampoline = AttitudePositionEstimatorEKF_task_main_trampoline;
		_aekf->task_main = AttitudePositionEstimatorEKF_task_main;
		_aekf->check_filter_state = AttitudePositionEstimatorEKF_check_filter_state;
		_aekf->publishAttitude = AttitudePositionEstimatorEKF_publishAttitude;
		_aekf->publishLocalPosition = AttitudePositionEstimatorEKF_publishLocalPosition;
		_aekf->publishGlobalPosition = AttitudePositionEstimatorEKF_publishGlobalPosition;
		_aekf->publishWindEstimate = AttitudePositionEstimatorEKF_publishWindEstimate;
		_aekf->updateSensorFusion = AttitudePositionEstimatorEKF_updateSensorFusion;
		_aekf->initializeGPS = AttitudePositionEstimatorEKF_initializeGPS;
		_aekf->initReferencePosition = AttitudePositionEstimatorEKF_initReferencePosition;
		_aekf->pollData = AttitudePositionEstimatorEKF_pollData;

		_aekf->_task_should_exit = false,
		_aekf->_task_running = false,
		_aekf->_estimator_task = -1,
	 */
	/* subscriptions */
	/*		_aekf->_sensor_combined_sub = -1,
			_aekf->_distance_sub = -1,
			_aekf->_airspeed_sub = -1,
			_aekf->_baro_sub = -1,
			_aekf->_gps_sub = -1,
			_aekf->_vstatus_sub = -1,
			_aekf->_params_sub = -1,
			_aekf->_manual_control_sub = -1,
			_aekf->_mission_sub = -1,
			_aekf->_home_sub = -1,
			_aekf->_landDetectorSub = -1,
			_aekf->_armedSub = -1,
	 */
	/* publications */
	/*		_aekf->_att_pub = 0,
			_aekf->_global_pos_pub = 0,
			_aekf->_local_pos_pub = 0,
			_aekf->_estimator_status_pub = 0,
			_aekf->_wind_pub = 0,
	 */
	/*_aekf->_att = {0} ,
	  _aekf->_gyro = ;
	  _aekf->_accel = {},
	  _aekf->_mag = {},
	  _aekf->_airspeed = {},
	  _aekf->_baro = {},
	  _aekf->_vstatus = {},
	  _aekf->_global_pos = {},
	  _aekf->_local_pos = {},
	  _aekf->_gps = {},
	  _aekf->_wind = {},
	  _aekf->_distance = {},
	  _aekf->_landDetector = {},
	  _aekf->_armed = {},

	  _aekf->_gyro_offsets = {},
	  _aekf->_accel_offsets = {},
	  _aekf->_mag_offsets = {},

	  _aekf->_sensor_combined = {},

	  _aekf->_pos_ref{},
	 */ //_aekf->_filter_ref_offset = 0.0f,
	//		  _aekf->_baro_gps_offset = 0.0f,

	/* performance counters */

	/*
	   _aekf->_loop_perf(perf_alloc(PC_ELAPSED, "ekf_att_pos_estimator")),
	   _aekf->_loop_intvl(perf_alloc(PC_INTERVAL, "ekf_att_pos_est_interval")),
	   _aekf->_perf_gyro(perf_alloc(PC_INTERVAL, "ekf_att_pos_gyro_upd")),
	   _aekf->_perf_mag(perf_alloc(PC_INTERVAL, "ekf_att_pos_mag_upd")),
	   _aekf->_perf_gps(perf_alloc(PC_INTERVAL, "ekf_att_pos_gps_upd")),
	   _aekf->_perf_baro(perf_alloc(PC_INTERVAL, "ekf_att_pos_baro_upd")),
	   _aekf->_perf_airspeed(perf_alloc(PC_INTERVAL, "ekf_att_pos_aspd_upd")),
	   _aekf->_perf_reset(perf_alloc(PC_COUNT, "ekf_att_pos_reset")),
	 */
	/* states */
	/*		_aekf->_gps_alt_filt = 0.0f,
			_aekf->_baro_alt_filt = 0.0f,
			_aekf->_covariancePredictionDt = 0.0f,
			_aekf->_gpsIsGood = false,
			_aekf->_previousGPSTimestamp = 0,
			_aekf->_baro_init = false,
			_aekf->_gps_initialized = false,
			_aekf->_filter_start_time = 0,
			_aekf->_last_sensor_timestamp = 0,
			_aekf->_last_run = 0,
			_aekf->_distance_last_valid = 0,
			_aekf->_gyro_valid = false,
			_aekf->_accel_valid = false,
			_aekf->_mag_valid = false,
			_aekf->_gyro_main = 0,
			_aekf->_accel_main = 0,
			_aekf->_mag_main = 0,
			_aekf->_ekf_logging = true,
			_aekf->_debug = 0,

			_aekf->_newHgtData = false,
			_aekf->_newAdsData = false,
			_aekf->_newDataMag = false,
			_aekf->_newRangeData = false,

			_aekf->_mavlink_fd = -1,
	// _aekf->_parameters{},
	// _aekf->_parameter_handles{},
	_ekf = NULL,

	// _aekf->_LP_att_P(100.0f, 10.0f),
	//  _aekf->_LP_att_Q(100.0f, 10.0f),
	//  _aekf->_LP_att_R(100.0f, 10.0f)
	// _aekf->_last_run = hrt_absolute_time();

	_aekf->_parameter_handles.vel_delay_ms = param_find("PE_VEL_DELAY_MS");
	_aekf->_parameter_handles.pos_delay_ms = param_find("PE_POS_DELAY_MS");
	_aekf->_parameter_handles.height_delay_ms = param_find("PE_HGT_DELAY_MS");
	_aekf->_parameter_handles.mag_delay_ms = param_find("PE_MAG_DELAY_MS");
	_aekf->_parameter_handles.tas_delay_ms = param_find("PE_TAS_DELAY_MS");
	_aekf->_parameter_handles.velne_noise = param_find("PE_VELNE_NOISE");
	_aekf->_parameter_handles.veld_noise = param_find("PE_VELD_NOISE");
	_aekf->_parameter_handles.posne_noise = param_find("PE_POSNE_NOISE");
	_aekf->_parameter_handles.posd_noise = param_find("PE_POSD_NOISE");
	_aekf->_parameter_handles.mag_noise = param_find("PE_MAG_NOISE");
	_aekf->_parameter_handles.gyro_pnoise = param_find("PE_GYRO_PNOISE");
	_aekf->_parameter_handles.acc_pnoise = param_find("PE_ACC_PNOISE");
	_aekf->_parameter_handles.gbias_pnoise = param_find("PE_GBIAS_PNOISE");
	_aekf->_parameter_handles.abias_pnoise = param_find("PE_ABIAS_PNOISE");
	_aekf->_parameter_handles.mage_pnoise = param_find("PE_MAGE_PNOISE");
	_aekf->_parameter_handles.magb_pnoise = param_find("PE_MAGB_PNOISE");
	_aekf->_parameter_handles.eas_noise = param_find("PE_EAS_NOISE");
	_aekf->_parameter_handles.pos_stddev_threshold = param_find("PE_POSDEV_INIT");
	 */
	/* indicate consumers that the current position data is not valid */
	//	_aekf->_gps.eph = 10000.0f;
	//	_aekf->_gps.epv = 10000.0f;

	/* fetch initial parameter values */
	parameters_update();

	/* get offsets */
	int fd, res;

	//for (unsigned s = 0; s < 3; s++) {
	unsigned s;
	/*	for (s = 0; s < 3; s++) {
		char str[30];
		(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);
		fd = px4_open(str, O_RDONLY);

		if (fd >= 0) {
		res = px4_ioctl(fd, GYROIOCGSCALE, (long unsigned int)&_aekf->_gyro_offsets[s]);
		px4_close(fd);

		if (res) {
		PX4_WARN("G%u SCALE FAIL", s);
		}
		}

		(void)sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, s);
		fd = px4_open(str, O_RDONLY);

		if (fd >= 0) {
		res = px4_ioctl(fd, ACCELIOCGSCALE, (long unsigned int)&_aekf->_accel_offsets[s]);
		px4_close(fd);

		if (res) {
		PX4_WARN("A%u SCALE FAIL", s);
		}
		}

		(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, s);
		fd = px4_open(str, O_RDONLY);

		if (fd >= 0) {
		res = px4_ioctl(fd, MAGIOCGSCALE, (long unsigned int)&_aekf->_mag_offsets[s]);
		px4_close(fd);

		if (res) {
		PX4_WARN("M%u SCALE FAIL", s);
		}
		}
		}*/
}

void AttitudePositionEstimatorEKF_destroy(struct AttitudePositionEstimatorEKF * _aekf)
{
	//if (_aekf->_estimator_task != -1) {

	/*task wakes up every 100ms or so at the longest */
	//		_aekf->_task_should_exit = true;

	/* wait for a second for the task to quit at our request */
	unsigned i = 0;

	//		do {
	/* wait 20ms */
	usleep(20000);

	/* if we have given up, kill it */
	if (++i > 50) {
		//		px4_task_delete(_aekf->_estimator_task);
		//break;
	}
	//		} while (_aekf->_estimator_task != -1);
	//}

	//delete _ekf;
	//free(_ekf);

	//AttitudePositionEstimatorEKF->g_estimator = NULL;
}

int AttitudePositionEstimatorEKF_enable_logging(struct AttitudePositionEstimatorEKF * _aekf,bool logging)//bool logging)
{
	//	_aekf->_ekf_logging = logging;

	return 0;
}

int AttitudePositionEstimatorEKF_parameters_update(struct AttitudePositionEstimatorEKF * _aekf)
{

	/*	param_get(_aekf->_parameter_handles.vel_delay_ms, &(_aekf->_parameters.vel_delay_ms));
		param_get(_aekf->_parameter_handles.pos_delay_ms, &(_aekf->_parameters.pos_delay_ms));
		param_get(_aekf->_parameter_handles.height_delay_ms, &(_aekf->_parameters.height_delay_ms));
		param_get(_aekf->_parameter_handles.mag_delay_ms, &(_aekf->_parameters.mag_delay_ms));
		param_get(_aekf->_parameter_handles.tas_delay_ms, &(_aekf->_parameters.tas_delay_ms));
		param_get(_aekf->_parameter_handles.velne_noise, &(_aekf->_parameters.velne_noise));
		param_get(_aekf->_parameter_handles.veld_noise, &(_aekf->_parameters.veld_noise));
		param_get(_aekf->_parameter_handles.posne_noise, &(_aekf->_parameters.posne_noise));
		param_get(_aekf->_parameter_handles.posd_noise, &(_aekf->_parameters.posd_noise));
		param_get(_aekf->_parameter_handles.mag_noise, &(_aekf->_parameters.mag_noise));
		param_get(_aekf->_parameter_handles.gyro_pnoise, &(_aekf->_parameters.gyro_pnoise));
		param_get(_aekf->_parameter_handles.acc_pnoise, &(_aekf->_parameters.acc_pnoise));
		param_get(_aekf->_parameter_handles.gbias_pnoise, &(_aekf->_parameters.gbias_pnoise));
		param_get(_aekf->_parameter_handles.abias_pnoise, &(_aekf->_parameters.abias_pnoise));
		param_get(_aekf->_parameter_handles.mage_pnoise, &(_aekf->_parameters.mage_pnoise));
		param_get(_aekf->_parameter_handles.magb_pnoise, &(_aekf->_parameters.magb_pnoise));
		param_get(_aekf->_parameter_handles.eas_noise, &(_aekf->_parameters.eas_noise));
		param_get(_aekf->_parameter_handles.pos_stddev_threshold, &(_aekf->_parameters.pos_stddev_threshold));
	 */
	/*	if (_ekf) {
	// _ekf->yawVarScale = 1.0f;
	// _ekf->windVelSigma = 0.1f;
	_ekf->dAngBiasSigma = _aekf->_parameters.gbias_pnoise;
	_ekf->dVelBiasSigma = _aekf->_parameters.abias_pnoise;
	_ekf->magEarthSigma = _aekf->_parameters.mage_pnoise;
	_ekf->magBodySigma  = _aekf->_parameters.magb_pnoise;
	// _ekf->gndHgtSigma  = 0.02f;
	_ekf->vneSigma = _aekf->_parameters.velne_noise;
	_ekf->vdSigma = _aekf->_parameters.veld_noise;
	_ekf->posNeSigma = _aekf->_parameters.posne_noise;
	_ekf->posDSigma = _aekf->_parameters.posd_noise;
	_ekf->magMeasurementSigma = _aekf->_parameters.mag_noise;
	_ekf->gyroProcessNoise = _aekf->_parameters.gyro_pnoise;
	_ekf->accelProcessNoise = _aekf->_parameters.acc_pnoise;
	_ekf->airspeedMeasurementSigma = _aekf->_parameters.eas_noise;
	_ekf->rngFinderPitch = 0.0f; // XXX base on SENS_BOARD_Y_OFF
	}
	 */
	return 1;//OK
}

void AttitudePositionEstimatorEKF_vehicle_status_poll(struct AttitudePositionEstimatorEKF * _aekf)
{
	_Bool vstatus_updated = -1;

	/* Check HIL state if vehicle status has changed */
	/*orb_check(_aekf->_vstatus_sub, &vstatus_updated);

	  if (vstatus_updated) {

	  orb_copy(ORB_ID(vehicle_status), _aekf->_vstatus_sub, &_aekf->_vstatus);

	//Tell EKF that the vehicle is a fixed wing or multi-rotor
	AttPosEKF_setIsFixedWing(_ekf,!_aekf->_vstatus.is_rotary_wing);
	}*/
}

void AttitudePositionEstimatorEKF_control_update(struct AttitudePositionEstimatorEKF* control_update)
{

}



int AttitudePositionEstimatorEKF_check_filter_state(struct AttitudePositionEstimatorEKF * _aekf)
{
	/*
	 *    CHECK IF THE INPUT DATA IS SANE
	 */

	struct ekf_status_report ekf_report;

	int check = 0;//AttPosEKF_CheckAndBound(_ekf, &ekf_report);

	int checkrand=0;
	//For Test//
	//Frama_C 
	checkrand=nondet_int();

	if(checkrand==1){
		check=1;///
	}
	//For Test//

	const char *const feedback[] = { 0,
		"NaN in states, resetting",
		"stale sensor data, resetting",
		"got initial position lock",
		"excessive gyro offsets",
		"velocity diverted, check accel config",
		"excessive covariances",
		"unknown condition, resetting"
	};

	// Print out error condition
	if (check) {
		unsigned warn_index = (unsigned)(check);
		unsigned max_warn_index = (sizeof(feedback) / sizeof(feedback[0]));

		if (max_warn_index < warn_index) {
			warn_index = max_warn_index;
		}

		// Do not warn about accel offset if we have no position updates
		//if (!(warn_index == 5 && _ekf->staticMode)) {
		//	PX4_WARN("reset: %s", feedback[warn_index]);
		//	mavlink_log_critical(_aekf->_mavlink_fd, "[ekf check] %s", feedback[warn_index]);
		//}
	}

	struct estimator_status_s rep;

	memset(&rep, 0, sizeof(rep));

	//ekf_report.error = true;
	// If error flag is set, we got a filter reset
	//	if (check && ekf_report.error) {
	// Count the reset condition
	//perf_count(_aekf->_perf_reset);
	// GPS is in scaled integers, convert
	//double lat = _aekf->_gps.lat / 1.0e7;
	//double lon = _aekf->_gps.lon / 1.0e7;
	//float gps_alt = _aekf->_gps.alt / 1e3f;

	// Set up height correctly
	//orb_copy(ORB_ID(sensor_baro), _aekf->_baro_sub, &_aekf->_baro);

	//	AttitudePositionEstimatorEKF_initReferencePosition(_aekf,_aekf->_gps.timestamp_position, lat, lon, gps_alt, _aekf->_baro.altitude);

	//	} else if (_aekf->_ekf_logging) {
	AttPosEKF_GetFilterState(_ekf,&ekf_report);
	//	}

	//if (_aekf->_ekf_logging || check) {/////////////For Test//origin
	//	if(check){///delete

	//rep.timestamp = hrt_absolute_time();

	/* For Test
	   rep.nan_flags |= (((uint8_t)ekf_report.angNaN)		<< 0);
	   rep.nan_flags |= (((uint8_t)ekf_report.summedDelVelNaN)	<< 1);
	   rep.nan_flags |= (((uint8_t)ekf_report.KHNaN)		<< 2);
	   rep.nan_flags |= (((uint8_t)ekf_report.KHPNaN)		<< 3);
	   rep.nan_flags |= (((uint8_t)ekf_report.PNaN)		<< 4);
	   rep.nan_flags |= (((uint8_t)ekf_report.covarianceNaN)	<< 5);
	   rep.nan_flags |= (((uint8_t)ekf_report.kalmanGainsNaN)	<< 6);
	   rep.nan_flags |= (((uint8_t)ekf_report.statesNaN)	<< 7);

	   rep.health_flags |= (((uint8_t)ekf_report.velHealth)	<< 0);
	   rep.health_flags |= (((uint8_t)ekf_report.posHealth)	<< 1);
	   rep.health_flags |= (((uint8_t)ekf_report.hgtHealth)	<< 2);
	   rep.health_flags |= (((uint8_t)!ekf_report.gyroOffsetsExcessive)	<< 3);
	   rep.health_flags |= (((uint8_t)ekf_report.onGround)	<< 4);
	   rep.health_flags |= (((uint8_t)ekf_report.staticMode)	<< 5);
	   rep.health_flags |= (((uint8_t)ekf_report.useCompass)	<< 6);
	   rep.health_flags |= (((uint8_t)ekf_report.useAirspeed)	<< 7);

	   rep.timeout_flags |= (((uint8_t)ekf_report.velTimeout)	<< 0);
	   rep.timeout_flags |= (((uint8_t)ekf_report.posTimeout)	<< 1);
	   rep.timeout_flags |= (((uint8_t)ekf_report.hgtTimeout)	<< 2);
	   rep.timeout_flags |= (((uint8_t)ekf_report.imuTimeout)	<< 3);

	 */

	//if (_aekf->_debug > 10) {
	//	if (rep.health_flags < ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))) {
	/*	PX4_INFO("health: VEL:%s POS:%s HGT:%s OFFS:%s",
		((rep.health_flags & (1 << 0)) ? "OK" : "ERR"),
		((rep.health_flags & (1 << 1)) ? "OK" : "ERR"),
		((rep.health_flags & (1 << 2)) ? "OK" : "ERR"),
		((rep.health_flags & (1 << 3)) ? "OK" : "ERR"));
		mw *///}

	//		if (rep.timeout_flags) {
	/*	PX4_INFO("timeout: %s%s%s%s",
		((rep.timeout_flags & (1 << 0)) ? "VEL " : ""),
		((rep.timeout_flags & (1 << 1)) ? "POS " : ""),
		((rep.timeout_flags & (1 << 2)) ? "HGT " : ""),
		((rep.timeout_flags & (1 << 3)) ? "IMU " : ""));
		mw *///}
	//}
	// Copy all states or at least all that we can fit
	//For Test size_t ekf_n_states = ekf_report.n_states;
	//For Test size_t max_states = (sizeof(rep.states) / sizeof(rep.states[0]));
	//For Test rep.n_states = (ekf_n_states < max_states) ? ekf_n_states : max_states;

	// Copy diagonal elemnts of covariance matrix
	float covariances[28];
	//AttPosEKF_get_covariance(_ekf,covariances);

	size_t i;
	//for (i = 0; i < rep.n_states; i++) {
	//rep.states[i] = ekf_report.states[i];
	//	rep.covariances[i] = covariances[i];
	//}


	//For Test

	//Frama_C  
	int _estimator_status_pub_rand=0;
	_estimator_status_pub_rand=nondet_int();

	if(_estimator_status_pub_rand==1){
		//	_aekf->_estimator_status_pub=1;
	}


	//	if (_aekf->_estimator_status_pub != 0) {////!=0
	orb_publish("estimator_status", sizeof(rep));

	//	} else {
	//_aekf->_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &rep);
	//	}


	return check;

}
void AttitudePositionEstimatorEKF_task_main_trampoline(struct AttitudePositionEstimatorEKF * _aekf, int argc, char *argv[])
{
	//	g_estimator->task_main(_aekf);
}

void AttitudePositionEstimatorEKF_task_main(struct AttitudePositionEstimatorEKF * _aekf)
{

	//call constructor
	AttitudePositionEstimatorEKF_constructor(_aekf);

	_ekf = (struct AttPosEKF*)malloc(sizeof(struct AttPosEKF));
	AttPosEKF_constructor(_ekf);
	_lpf = (struct LowPassFilter2p*)malloc(sizeof(struct LowPassFilter2p));
	//LowPassFilter2p_constructor(_lpf,1.0f,1.0f);
	//_aekf->_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

	if (!_ekf) {
		PX4_ERR("OUT OF MEM!");
		return;
	}

	//_aekf->_filter_start_time = hrt_absolute_time();

	/*
	 * do subscriptions
	 */
	/*_aekf->_distance_sub = orb_subscribe(ORB_ID(distance_sensor));
	  _aekf->_baro_sub = orb_subscribe_multi(ORB_ID(sensor_baro), 0);
	  _aekf->_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	  _aekf->_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	  _aekf->_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	  _aekf->_params_sub = orb_subscribe(ORB_ID(parameter_update));
	  _aekf->_home_sub = orb_subscribe(ORB_ID(home_position));
	  _aekf->_landDetectorSub = orb_subscribe(ORB_ID(vehicle_land_detected));
	  _aekf->_armedSub = orb_subscribe(ORB_ID(actuator_armed));
	 */
	/* rate limit vehicle status updates to 5Hz */
	//	orb_set_interval(_aekf->_vstatus_sub, 200);

	//	_aekf->_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	/* XXX remove this!, BUT increase the data buffer size! */
	//	orb_set_interval(_aekf->_sensor_combined_sub, 9);

	/* sets also parameters in the EKF object */
	parameters_update();

	/* wakeup source(s) */
	//struct px4_pollfd_struct_t fds[2];
	struct pollfd fds[2];
	/* Setup of loop */
	//	fds[0].fd = _aekf->_params_sub;
	//	fds[0].events = POLLIN;
	//	fds[0].revents = POLLIN;

	//	fds[1].fd = _aekf->_sensor_combined_sub;
	//	fds[1].events = POLLIN;
	//	fds[1].revents = POLLIN;

	//	_aekf->_gps.vel_n_m_s = 0.0f;
	//	_aekf->_gps.vel_e_m_s = 0.0f;
	//	_aekf->_gps.vel_d_m_s = 0.0f;

	//	_aekf->_task_running = true;

	//while (!_aekf->_task_should_exit) {
	int idx=0;
	for(idx=0;idx<5;idx++){
		//	while(1){
		/* wait for up to 100ms for data */
		int loopcnt=0;
		loopcnt=rand()%2;

		if(loopcnt==1) break;	//_aekf->_task_shoule_exit bool 타입 동작 추상화

		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);


		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			//		continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			//warn("POLL ERR %d, %d", pret, errno);
			//		continue;
		}

		//		perf_begin(_aekf->_loop_perf);
		//		perf_count(_aekf->_loop_intvl);

		/* only update parameters if they changed */
		//if (fds[0].revents & POLLIN) {
		/* read from param to clear updated flag */
		struct parameter_update_s update;
		//			orb_copy(ORB_ID(parameter_update), _aekf->_params_sub, &update);

		/* update parameters from storage */
		parameters_update();
		//	}

		/* only run estimator if gyro updated */
		//	if (fds[1].revents & POLLIN) {

		/* check vehicle status for changes to publication state */
		_Bool prev_hil = -1;
		//			prev_hil = (_aekf->_vstatus.hil_state == _aekf->_vstatus.HIL_STATE_ON);
		AttitudePositionEstimatorEKF_vehicle_status_poll(_aekf);

		//			perf_count(_aekf->_perf_gyro);

		/* Reset baro reference if switching to HIL, reset sensor states */
		//		if (!prev_hil && (_aekf->_vstatus.hil_state == _aekf->_vstatus.HIL_STATE_ON)) {
		/* system is in HIL now, wait for measurements to come in one last round */
		usleep(60000);

		/* now read all sensor publications to ensure all real sensor data is purged */
		//				orb_copy(ORB_ID(sensor_combined), _aekf->_sensor_combined_sub, &_aekf->_sensor_combined);

		/* set sensors to de-initialized state */
		/*				_aekf->_gyro_valid = false;
						_aekf->_accel_valid = false;
						_aekf->_mag_valid = false;

						_aekf->_baro_init = false;
						_aekf->_gps_initialized = false;

		//_aekf->_last_sensor_timestamp = hrt_absolute_time();
		_aekf->_last_run = _aekf->_last_sensor_timestamp;

		//AttPosEKF_ZeroVariables(_ekf);
		_ekf->dtIMU = 0.01f;
		_aekf->_filter_start_time = _aekf->_last_sensor_timestamp;
		 */
		/* now skip this loop and get data on the next one, which will also re-init the filter */
		//			continue;
		//		}

		/**
		 *    PART ONE: COLLECT ALL DATA
		 **/
		AttitudePositionEstimatorEKF_pollData(_aekf);

		/*
		 *    CHECK IF ITS THE RIGHT TIME TO RUN THINGS ALREADY
		 */
		/*	if (hrt_elapsed_time(&_aekf->_filter_start_time) < FILTER_INIT_DELAY) {
			continue;
			}
		 */
		/**
		 *    PART TWO: EXECUTE THE FILTER
		 *
		 *    We run the filter only once all data has been fetched
		 **/

		// To enter if condition.

		/*			_aekf->_baro_init = true;
					_aekf->_gyro_valid = true; 
					_aekf->_accel_valid = true;
					_aekf->_mag_valid = true;
					_ekf->statesInitialised = true;*/
		//		if (_aekf->_baro_init && _aekf->_gyro_valid && _aekf->_accel_valid && _aekf->_mag_valid) {

		// maintain filtered baro and gps altitudes to calculate weather offset
		// baro sample rate is ~70Hz and measurement bandwidth is high
		// gps sample rate is 5Hz and altitude is assumed accurate when averaged over 30 seconds
		// maintain heavily filtered values for both baro and gps altitude
		// Assume the filtered output should be identical for both sensors

		//			if (_aekf->_gpsIsGood) {
		//					_aekf->_baro_gps_offset = _aekf->_baro_alt_filt - _aekf->_gps_alt_filt;
		//			}
		//				if (hrt_elapsed_time(&_last_debug_print) >= 5e6) {
		//					_last_debug_print = hrt_absolute_time();
		//					perf_print_counter(_perf_baro);
		//					perf_reset(_perf_baro);
		//					PX4_INFO("gpsoff: %5.1f, baro_alt_filt: %6.1f, gps_alt_filt: %6.1f, gpos.alt: %5.1f, lpos.z: %6.1f",
		//							(double)_baro_gps_offset,
		//							(double)_baro_alt_filt,
		//							(double)_gps_alt_filt,
		//							(double)_global_pos.alt,
		//							(double)_local_pos.z);
		//				}

		/* Initialize the filter first */
		//			if (!_ekf->statesInitialised) {
		// North, East Down position (m)
		float initVelNED[3] = {0.0f, 0.0f, 0.0f};

		/*					_ekf->posNE[0] = 0.0f;
							_ekf->posNE[1] = 0.0f;

							_aekf->_local_pos.ref_alt = 0.0f;
							_aekf->_baro_gps_offset = 0.0f;
							_aekf->_baro_alt_filt = _aekf->_baro.altitude;

							AttPosEKF_InitialiseFilter(_ekf,initVelNED, 0.0, 0.0, 0.0f, 0.0f);

							_aekf->_filter_ref_offset = -_aekf->_baro.altitude;
		 */
		//PX4_INFO("filter ref off: baro_alt: %8.4f", (double)_aekf->_filter_ref_offset);

		//			} else {

		//					if (!_aekf->_gps_initialized && _aekf->_gpsIsGood) {
		//AttitudePositionEstimatorEKF_initializeGPS(_aekf);
		//						continue;
		//					}

		// Check if on ground - status is used by covariance prediction
		//AttPosEKF_setOnGround(_ekf,_aekf->_landDetector.landed);

		// We're apparently initialized in this case now
		// check (and reset the filter as needed)
		int check = AttitudePositionEstimatorEKF_check_filter_state(_aekf);

		if (check) {
			// Let the system re-initialize itself
			continue;
		}

		// Run EKF data fusion steps
		//AttitudePositionEstimatorEKF_updateSensorFusion(_aekf,_aekf->_gpsIsGood, _aekf->_newDataMag, _aekf->_newRangeData, _aekf->_newHgtData, _aekf->_newAdsData);//no pub func

		// Publish attitude estimations
		AttitudePositionEstimatorEKF_publishAttitude(_aekf);
		
		// Publish Local Position estimations
		AttitudePositionEstimatorEKF_publishLocalPosition(_aekf);

		// Publish Global Position, it will have a large uncertainty
		// set if only altitude is known
		AttitudePositionEstimatorEKF_publishGlobalPosition(_aekf);

		// Publish wind estimates
		//For Test
		//if (hrt_elapsed_time(&_aekf->_wind.timestamp) > 99000) {
		AttitudePositionEstimatorEKF_publishWindEstimate(_aekf);
		//}
		
		//	}
	}

	//	}

	//		perf_end(_aekf->_loop_perf);
	//	}

	//	_aekf->_task_running = false;

	//	_aekf->_estimator_task = -1;
	return;
	}

	void AttitudePositionEstimatorEKF_initReferencePosition(struct AttitudePositionEstimatorEKF * _aekf, hrt_abstime timestamp,
			double lat, double lon, float gps_alt, float baro_alt)
	{
		// Reference altitude
		/*if (PX4_ISFINITE(_ekf->states[9])) {
		  _aekf->_filter_ref_offset = _ekf->states[9];
		  } else if (PX4_ISFINITE(-_ekf->hgtMea)) {
		  _aekf->_filter_ref_offset = -_ekf->hgtMea;
		  } else {
		  _aekf->_filter_ref_offset = -_aekf->_baro.altitude;
		  }

		// init filtered gps and baro altitudes
		_aekf->_gps_alt_filt = gps_alt;
		_aekf->_baro_alt_filt = baro_alt;

		// Initialize projection
		_aekf->_local_pos.ref_lat = lat;
		_aekf->_local_pos.ref_lon = lon;
		_aekf->_local_pos.ref_alt = gps_alt;
		_aekf->_local_pos.ref_timestamp = timestamp;

		//map_projection_init(&_aekf->_pos_ref, lat, lon);
		mavlink_log_info(_aekf->_mavlink_fd, "[ekf] ref: LA %.4f,LO %.4f,ALT %.2f", lat, lon, (double)gps_alt);
		 */	}

	void AttitudePositionEstimatorEKF_initializeGPS(struct AttitudePositionEstimatorEKF * _aekf)
	{
		// GPS is in scaled integers, convert
		double lat;// = _aekf->_gps.lat / 1.0e7;
		double lon;// = _aekf->_gps.lon / 1.0e7;
		float gps_alt;// = _aekf->_gps.alt / 1e3f;

		// Set up height correctly
		/*orb_copy(ORB_ID(sensor_baro), _aekf->_baro_sub, &_aekf->_baro);

		  _ekf->baroHgt = _aekf->_baro.altitude;
		  _ekf->hgtMea = _ekf->baroHgt;

		// Set up position variables correctly
		_ekf->GPSstatus = _aekf->_gps.fix_type;

		_ekf->gpsLat = radians(lat);
		_ekf->gpsLon = radians(lon) - M_PI;
		_ekf->gpsHgt = gps_alt;
		 */
		// Look up mag declination based on current position
		float declination = radians(get_mag_declination(lat, lon));

		float initVelNED[3];
		/*		initVelNED[0] = _aekf->_gps.vel_n_m_s;
				initVelNED[1] = _aekf->_gps.vel_e_m_s;
				initVelNED[2] = _aekf->_gps.vel_d_m_s;
		 */
		//AttPosEKF_InitialiseFilter(_ekf,initVelNED, radians(lat), radians(lon) - M_PI, gps_alt, declination);

		//		AttitudePositionEstimatorEKF_initReferencePosition(_aekf,_aekf->_gps.timestamp_position, lat, lon, gps_alt, _aekf->_baro.altitude);

		/*#if 0
		  PX4_INFO("HOME/REF: LA %8.4f,LO %8.4f,ALT %8.2f V: %8.4f %8.4f %8.4f", lat, lon, (double)gps_alt,
		  (double)_ekf->velNED[0], (double)_ekf->velNED[1], (double)_ekf->velNED[2]);
		  PX4_INFO("BARO: %8.4f m / ref: %8.4f m / gps offs: %8.4f m", (double)_ekf->baroHgt, (double)_baro_ref,
		  (double)_filter_ref_offset);
		  PX4_INFO("GPS: eph: %8.4f, epv: %8.4f, declination: %8.4f", (double)_gps.eph, (double)_gps.epv,
		  (double)math::degrees(declination));
#endif
		 */
		//		_aekf->_gps_initialized = true;
	}

	void AttitudePositionEstimatorEKF_publishAttitude(struct AttitudePositionEstimatorEKF * _aekf)
	{
		int i;
		int j;
		// Output results
		//	Quaternion q(_ekf->states[0], _ekf->states[1], _ekf->states[2], _ekf->states[3]);
		//	Matrix<3, 3> R = q.to_dcm();
		//	Vector<3> euler = R.to_euler();

		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				//PX4_R(_att.R, i, j) = R(i, j);
			}
		}
		//
		struct vehicle_attitude_s _att;

		/*	_aekf->_att.timestamp = _aekf->_last_sensor_timestamp;
			_aekf->_att.q[0] = _ekf->states[0];
			_aekf->_att.q[1] = _ekf->states[1];
			_aekf->_att.q[2] = _ekf->states[2];
			_aekf->_att.q[3] = _ekf->states[3];
			_aekf->_att.q_valid = true;
			_aekf->_att.R_valid = true;

			_aekf->_att.timestamp =_aekf->_last_sensor_timestamp;
			_aekf->_att.roll = euler(0);
			_aekf->_att.pitch = euler(1);
			_aekf->_att.yaw = euler(2);

			_aekf->_att.rollspeed = LowPassFilter2p_apply(_lpf,_ekf->angRate.x) - _ekf->states[10] / _ekf->dtIMUfilt;
			_aekf->_att.pitchspeed = LowPassFilter2p_apply(_lpf,_ekf->angRate.y) - _ekf->states[11] / _ekf->dtIMUfilt;
			_aekf->_att.yawspeed = LowPassFilter2p_apply(_lpf,_ekf->angRate.z) - _ekf->states[12] / _ekf->dtIMUfilt;

		// gyro offsets
		_aekf->_att.rate_offsets[0] = _ekf->states[10] / _ekf->dtIMUfilt;
		_aekf->_att.rate_offsets[1] = _ekf->states[11] / _ekf->dtIMUfilt;
		_aekf->_att.rate_offsets[2] = _ekf->states[12] / _ekf->dtIMUfilt;
		 */
		//Modify
		//_aekf->_att_pub = 1;

		int _att_pub_rand=0;

		//Frama_c 
		_att_pub_rand=nondet_int();

		if(_att_pub_rand==1){

			//_aekf->_att_pub = 1;

		}	


		//int hjsize;
		//hjsize=_aekf->_att;
		/* lazily publish the attitude only once available */
		//if (_aekf->_att_pub != 0) {
		/* publish the attitude setpoint */
		orb_publish("vehicle_attitude", sizeof(_aekf->_att));
		orb_publish("vehicle_attitude", sizeof(_aekf->_att));
		orb_publish("estimator_status",sizeof(_aekf->_att));////////////////////////////////////test

		//	} else {
		/* advertise and publish */
		//		_aekf->_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &_aekf->_att);
		//	}
	}

	void AttitudePositionEstimatorEKF_publishLocalPosition(struct AttitudePositionEstimatorEKF * _aekf)
	{
		/*_aekf->_local_pos.timestamp = _aekf->_last_sensor_timestamp;
		  _aekf->_local_pos.x = _ekf->states[7];
		  _aekf->_local_pos.y = _ekf->states[8];

		// XXX need to announce change of Z reference somehow elegantly
		_aekf->_local_pos.z = _ekf->states[9] - _aekf->_filter_ref_offset;
		//_local_pos.z_stable = _ekf->states[9];

		_aekf->_local_pos.vx = _ekf->states[4];
		_aekf->_local_pos.vy = _ekf->states[5];
		_aekf->_local_pos.vz = _ekf->states[6];

		_aekf->_local_pos.xy_valid = _aekf->_gps_initialized && _aekf->_gpsIsGood;
		_aekf->_local_pos.z_valid = true;
		_aekf->_local_pos.v_xy_valid = _aekf->_gps_initialized && _aekf->_gpsIsGood;
		_aekf->_local_pos.v_z_valid = true;
		_aekf->_local_pos.xy_global = _aekf->_gps_initialized; //TODO: Handle optical flow mode here

		_aekf->_local_pos.z_global = false;
		_aekf->_local_pos.yaw = _aekf->_att.yaw;

		if (!PX4_ISFINITE(_aekf->_local_pos.x) ||
		!PX4_ISFINITE(_aekf->_local_pos.y) ||
		!PX4_ISFINITE(_aekf->_local_pos.z) ||
		!PX4_ISFINITE(_aekf->_local_pos.vx) ||
		!PX4_ISFINITE(_aekf->_local_pos.vy) ||
		!PX4_ISFINITE(_aekf->_local_pos.vz))
		{
		// bad data, abort publication
		return;
		}
		 */
		int _local_pos_pub_rand=0;

		//frama_c 
		_local_pos_pub_rand=nondet_int();//

		if(_local_pos_pub_rand==1){
			//			_aekf->_local_pos_pub = 1;
		}


		/* lazily publish the local position only once available */
		//	if (_aekf->_local_pos_pub != 0) {
		/* publish the attitude setpoint */
		orb_publish("vehicle_local_position", sizeof(_aekf->_local_pos));

		//	} else {
		/* advertise and publish */
		//		_aekf->_local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &_aekf->_local_pos);
		//	}
	}

	void AttitudePositionEstimatorEKF_publishGlobalPosition(struct AttitudePositionEstimatorEKF * _aekf)
	{
		//_aekf->_global_pos.timestamp = _aekf->_local_pos.timestamp;

		/*if (_aekf->_local_pos.xy_global) {
		  float est_lat, est_lon;
		  map_projection_reproject(&_aekf->_pos_ref, _aekf->_local_pos.x, _aekf->_local_pos.y, &est_lat, &est_lon);
		  _aekf->_global_pos.lat = est_lat;
		  _aekf->_global_pos.lon = est_lon;
		  _aekf->_global_pos.time_utc_usec = _aekf->_gps.time_utc_usec;
		  } else {
		  _aekf->_global_pos.lat = 0.0;
		  _aekf->_global_pos.lon = 0.0;
		  _aekf->_global_pos.time_utc_usec = 0;
		  }

		  if (_aekf->_local_pos.v_xy_valid) {
		  _aekf->_global_pos.vel_n = _aekf->_local_pos.vx;
		  _aekf->_global_pos.vel_e = _aekf->_local_pos.vy;

		  } else {
		  _aekf->_global_pos.vel_n = 0.0f;
		  _aekf->_global_pos.vel_e = 0.0f;
		  }
		 */
		/* local pos alt is negative, change sign and add alt offsets */
		/*		_aekf->_global_pos.alt = (-_aekf->_local_pos.z) - _aekf->_filter_ref_offset -  _aekf->_baro_gps_offset;

				if (_aekf->_local_pos.v_z_valid) {
				_aekf->_global_pos.vel_d = _aekf->_local_pos.vz;
				} else {
				_aekf->_global_pos.vel_d = 0.0f;
				}
		 */
		/* terrain altitude */
		/*		_aekf->_global_pos.terrain_alt = _ekf->hgtRef - _ekf->flowStates[1];
				_aekf->_global_pos.terrain_alt_valid = (_aekf->_distance_last_valid > 0) &&
				(hrt_elapsed_time(&_aekf->_distance_last_valid) < 20 * 1000 * 1000);

				_aekf->_global_pos.yaw = _aekf->_local_pos.yaw;
				_aekf->_global_pos.eph = _aekf->_gps.eph;
				_aekf->_global_pos.epv = _aekf->_gps.epv;

				if (!PX4_ISFINITE(_aekf->_global_pos.lat) ||
				!PX4_ISFINITE(_aekf->_global_pos.lon) ||
				!PX4_ISFINITE(_aekf->_global_pos.alt) ||
				!PX4_ISFINITE(_aekf->_global_pos.vel_n) ||
				!PX4_ISFINITE(_aekf->_global_pos.vel_e) ||
				!PX4_ISFINITE(_aekf->_global_pos.vel_d))
				{
		// bad data, abort publication
		return;
		}
		 */
		//For Test ????????

		//For Test	_aekf->_global_pos_pub=1;
		//////////

		int _global_pos_pub_rand=0;
		//frama_c 
		_global_pos_pub_rand=nondet_int();//

		if(_global_pos_pub_rand==1){
			//			_aekf->_global_pos_pub=1;
		}


		/* lazily publish the global position only once available */
		//	if (_aekf->_global_pos_pub != 0) {
		/* publish the global position */
		orb_publish("vehicle_global_position",sizeof(_aekf->_global_pos));

		//	} else {
		/* advertise and publish */
		//		_aekf->_global_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &_aekf->_global_pos);
		//	}
	}

	void AttitudePositionEstimatorEKF_publishWindEstimate(struct AttitudePositionEstimatorEKF * _aekf)
	{
		/*	_aekf->_wind.timestamp = _aekf->_global_pos.timestamp;
			_aekf->_wind.windspeed_north = _ekf->states[14];
			_aekf->_wind.windspeed_east = _ekf->states[15];

		// XXX we need to do something smart about the covariance here
		// but we default to the estimate covariance for now
		_aekf->_wind.covariance_north = _ekf->P[14][14];
		_aekf->_wind.covariance_east = _ekf->P[15][15];
		 */

		int _wind_pub_rand=0;
		_wind_pub_rand=nondet_int();

		if(_wind_pub_rand=1){
			//		_aekf->_wind_pub=1;//For Test
		}
		/* lazily publish the wind estimate only once available */
		//	if (_aekf->_wind_pub != 0) {
		/* publish the wind estimate */
		orb_publish("wind_estimate", sizeof(_aekf->_wind));

		//	} else {
		/* advertise and publish */
		//		_aekf->_wind_pub = orb_advertise(ORB_ID(wind_estimate), &_aekf->_wind);
		//	}

	}

	void AttitudePositionEstimatorEKF_updateSensorFusion(struct AttitudePositionEstimatorEKF * _aekf, const bool fuseGPS, const bool fuseMag,
			const bool fuseRangeSensor, const bool fuseBaro, const bool fuseAirSpeed)
	{
		/*
		// Run the strapdown INS equations every IMU update
		//_ekf->UpdateStrapdownEquationsNED(_ekf);
		AttPosEKF_UpdateStrapdownEquationsNED(_ekf);
		// store the predicted states for subsequent use by measurement fusion
		//_ekf->StoreStates(_ekf,IMUmsec);
		AttPosEKF_StoreStates(_ekf,IMUmsec);

		// sum delta angles and time used by covariance prediction
		//_ekf->summedDelAng = operator_plus(_ekf->summedDelAng , _ekf->correctedDelAng);

		_ekf->summedDelAng = operator_plus(_ekf->summedDelAng , _ekf->correctedDelAng);
		_ekf->summedDelVel = operator_plus(_ekf->summedDelVel , _ekf->dVelIMU);
		_aekf->_covariancePredictionDt += _ekf->dtIMU;

		// perform a covariance prediction if the total delta angle has exceeded the limit
		// or the time limit will be exceeded at the next IMU update
		if ((_aekf->_covariancePredictionDt >= (_ekf->covTimeStepMax - _ekf->dtIMU))
		|| (_ekf->summedDelAng.length(_vec3f) > _ekf->covDelAngMax)) {
		AttPosEKF_CovariancePrediction(_ekf,_aekf->_covariancePredictionDt);
		//_ekf->summedDelAng.zero(_vec3f);
		//_ekf->summedDelAng.zero(_vec3f);
		Vector3f_zero(&_ekf->summedDelAng);
		Vector3f_zero(&_ekf->summedDelVel);
		_aekf->_covariancePredictionDt = 0.0f;
		}

		// Fuse GPS Measurements
		if (fuseGPS && _aekf->_gps_initialized) {
		// Convert GPS measurements to Pos NE, hgt and Vel NED

		// set fusion flags
		_ekf->fuseVelData = _aekf->_gps.vel_ned_valid;
		_ekf->fusePosData = true;

		// recall states stored at time of measurement after adjusting for delays
		AttPosEKF_RecallStates(_ekf,_ekf->statesAtVelTime, (IMUmsec - _aekf->_parameters.vel_delay_ms));
		AttPosEKF_RecallStates(_ekf,_ekf->statesAtPosTime, (IMUmsec - _aekf->_parameters.pos_delay_ms));

		// run the fusion step
		AttPosEKF_FuseVelposNED(_ekf);

		} else if (!_aekf->_gps_initialized) {

		// force static mode
		_ekf->staticMode = true;

		// Convert GPS measurements to Pos NE, hgt and Vel NED
		_ekf->velNED[0] = 0.0f;
		_ekf->velNED[1] = 0.0f;
		_ekf->velNED[2] = 0.0f;

		_ekf->posNE[0] = 0.0f;
		_ekf->posNE[1] = 0.0f;

		// set fusion flags
		_ekf->fuseVelData = true;
		_ekf->fusePosData = true;

		// recall states stored at time of measurement after adjusting for delays
		AttPosEKF_RecallStates(_ekf,_ekf->statesAtVelTime, (IMUmsec - _aekf->_parameters.vel_delay_ms));
		AttPosEKF_RecallStates(_ekf,_ekf->statesAtPosTime, (IMUmsec - _aekf->_parameters.pos_delay_ms));

		// run the fusion step
		AttPosEKF_FuseVelposNED(_ekf);

		} else {
		_ekf->fuseVelData = false;
		_ekf->fusePosData = false;
		}

		if (fuseBaro) {
			// Could use a blend of GPS and baro alt data if desired
			_ekf->hgtMea = _ekf->baroHgt;
			_ekf->fuseHgtData = true;

			// recall states stored at time of measurement after adjusting for delays
			AttPosEKF_RecallStates(_ekf,_ekf->statesAtHgtTime, (IMUmsec - _aekf->_parameters.height_delay_ms));

			// run the fusion step
			AttPosEKF_FuseVelposNED(_ekf);

		} else {
			_ekf->fuseHgtData = false;
		}

		// Fuse Magnetometer Measurements
		if (fuseMag) {
			_ekf->fuseMagData = true;
			AttPosEKF_RecallStates(_ekf,_ekf->statesAtMagMeasTime,
					(IMUmsec - _aekf->_parameters.mag_delay_ms)); // Assume 50 msec avg delay for magnetometer data

			_ekf->magstate.obsIndex = 0;
			AttPosEKF_FuseMagnetometer(_ekf);
			AttPosEKF_FuseMagnetometer(_ekf);
			AttPosEKF_FuseMagnetometer(_ekf);

		} else {
			_ekf->fuseMagData = false;
		}

		// Fuse Airspeed Measurements
		if (fuseAirSpeed && _aekf->_airspeed.true_airspeed_m_s > 5.0f) {
			_ekf->fuseVtasData = true;
			AttPosEKF_RecallStates(_ekf,_ekf->statesAtVtasMeasTime,
					(IMUmsec - _aekf->_parameters.tas_delay_ms)); // assume 100 msec avg delay for airspeed data
			AttPosEKF_FuseAirspeed(_ekf);

		} else {
			_ekf->fuseVtasData = false;
		}

		// Fuse Rangefinder Measurements
		if (fuseRangeSensor) {
			if (_ekf->Tnb.z.z > 0.9f) {
				// _ekf->rngMea is set in sensor readout already
				_ekf->fuseRngData = true;
				_ekf->fuseOptFlowData = false;
				AttPosEKF_RecallStates(_ekf,_ekf->statesAtRngTime, (IMUmsec - 100.0f));
				AttPosEKF_OpticalFlowEKF(_ekf);
				_ekf->fuseRngData = false;
			}
		}*/
	}

	int AttitudePositionEstimatorEKF_start(struct AttitudePositionEstimatorEKF * _aekf)
	{
		//ASSERT(_aekf->_estimator_task == -1);

		/* start the task */
		/*_aekf->_estimator_task = px4_task_spawn_cmd("ekf_att_pos_estimator",
		  SCHED_DEFAULT,
		  SCHED_PRIORITY_MAX - 40,
		  7500,
		//(px4_main_t)
		0,
		0);//&AttitudePositionEstimatorEKF_task_main_trampoline

		if (_aekf->_estimator_task < 0) {
		//warn("task start failed");
		return -errno;
		}
		 */
		return 1;//OK
	}

	void AttitudePositionEstimatorEKF_print_status(struct AttitudePositionEstimatorEKF * _aekf)
	{
		/*math::Quaternion q(_ekf->states[0], _ekf->states[1], _ekf->states[2], _ekf->states[3]);
		  math::Matrix<3, 3> R = q.to_dcm();
		  math::Vector<3> euler = R.to_euler();
		 */
		//PX4_INFO("attitude: roll: %8.4f, pitch %8.4f, yaw: %8.4f degrees\n",			(double)degrees(euler(0)), (double)degrees(euler(1)), (double)degrees(euler(2)));

		// State vector:
		// 0-3: quaternions (q0, q1, q2, q3)
		// 4-6: Velocity - m/sec (North, East, Down)
		// 7-9: Position - m (North, East, Down)
		// 10-12: Delta Angle bias - rad (X,Y,Z)
		// 13:    Delta Velocity Bias - m/s (Z)
		// 14-15: Wind Vector  - m/sec (North,East)
		// 16-18: Earth Magnetic Field Vector - gauss (North, East, Down)
		// 19-21: Body Magnetic Field Vector - gauss (X,Y,Z)

		/* PX4_INFO("dtIMU: %8.6f filt: %8.6f IMUmsec: %d", (double)_ekf->dtIMU, (double)_ekf->dtIMUfilt, (int)IMUmsec);
		   PX4_INFO("alt RAW: baro alt: %8.4f GPS alt: %8.4f", (double)_aekf->_baro.altitude, (double)_ekf->gpsHgt);
		   PX4_INFO("alt EST: local alt: %8.4f (NED), AMSL alt: %8.4f (ENU)", (double)(_aekf->_local_pos.z), (double)_aekf->_global_pos.alt);
		   PX4_INFO("filter ref offset: %8.4f baro GPS offset: %8.4f", (double)_aekf->_filter_ref_offset,
		   (double)_aekf->_baro_gps_offset);
		   PX4_INFO("dvel: %8.6f %8.6f %8.6f accel: %8.6f %8.6f %8.6f", (double)_ekf->dVelIMU.x, (double)_ekf->dVelIMU.y,
		   (double)_ekf->dVelIMU.z, (double)_ekf->accel.x, (double)_ekf->accel.y, (double)_ekf->accel.z);
		   PX4_INFO("dang: %8.4f %8.4f %8.4f dang corr: %8.4f %8.4f %8.4f" , (double)_ekf->dAngIMU.x, (double)_ekf->dAngIMU.y,
		   (double)_ekf->dAngIMU.z, (double)_ekf->correctedDelAng.x, (double)_ekf->correctedDelAng.y,
		   (double)_ekf->correctedDelAng.z);
		   PX4_INFO("states (quat)        [0-3]: %8.4f, %8.4f, %8.4f, %8.4f", (double)_ekf->states[0], (double)_ekf->states[1],
		   (double)_ekf->states[2], (double)_ekf->states[3]);
		   PX4_INFO("states (vel m/s)     [4-6]: %8.4f, %8.4f, %8.4f", (double)_ekf->states[4], (double)_ekf->states[5],
		   (double)_ekf->states[6]);
		   PX4_INFO("states (pos m)      [7-9]: %8.4f, %8.4f, %8.4f", (double)_ekf->states[7], (double)_ekf->states[8],
		   (double)_ekf->states[9]);
		   PX4_INFO("states (delta ang) [10-12]: %8.4f, %8.4f, %8.4f", (double)_ekf->states[10], (double)_ekf->states[11],
		   (double)_ekf->states[12]);
		 */
		/*	if (EKF_STATE_ESTIMATES == 23) {
			PX4_INFO("states (accel offs)   [13]: %8.4f", (double)_ekf->states[13]);
			PX4_INFO("states (wind)      [14-15]: %8.4f, %8.4f", (double)_ekf->states[14], (double)_ekf->states[15]);
			PX4_INFO("states (earth mag) [16-18]: %8.4f, %8.4f, %8.4f", (double)_ekf->states[16], (double)_ekf->states[17],		(double)_ekf->states[18]);
			PX4_INFO("states (body mag)  [19-21]: %8.4f, %8.4f, %8.4f", (double)_ekf->states[19], (double)_ekf->states[20],		(double)_ekf->states[21]);
			PX4_INFO("states (terrain)      [22]: %8.4f", (double)_ekf->states[22]);

			} else {
			PX4_INFO("states (wind)      [13-14]: %8.4f, %8.4f", (double)_ekf->states[13], (double)_ekf->states[14]);
			PX4_INFO("states (earth mag) [15-17]: %8.4f, %8.4f, %8.4f", (double)_ekf->states[15], (double)_ekf->states[16],
			(double)_ekf->states[17]);
			PX4_INFO("states (body mag)  [18-20]: %8.4f, %8.4f, %8.4f", (double)_ekf->states[18], (double)_ekf->states[19],
			(double)_ekf->states[20]);
			}
		 */
		/*	PX4_INFO("states: %s %s %s %s %s %s %s %s %s %s",
			(_ekf->statesInitialised) ? "INITIALIZED" : "NON_INIT",
			(_aekf->_landDetector.landed) ? "ON_GROUND" : "AIRBORNE",
			(_ekf->fuseVelData) ? "FUSE_VEL" : "INH_VEL",
			(_ekf->fusePosData) ? "FUSE_POS" : "INH_POS",
			(_ekf->fuseHgtData) ? "FUSE_HGT" : "INH_HGT",
			(_ekf->fuseMagData) ? "FUSE_MAG" : "INH_MAG",
			(_ekf->fuseVtasData) ? "FUSE_VTAS" : "INH_VTAS",
			(_ekf->useAirspeed) ? "USE_AIRSPD" : "IGN_AIRSPD",
			(_ekf->useCompass) ? "USE_COMPASS" : "IGN_COMPASS",
			(_ekf->staticMode) ? "STATIC_MODE" : "DYNAMIC_MODE");
		 */}

	void AttitudePositionEstimatorEKF_pollData(struct AttitudePositionEstimatorEKF * _aekf)
	{

		//Update arming status
		_Bool armedUpdate = -1;
		//orb_check(_aekf->_armedSub, &armedUpdate);

		if (armedUpdate) {
			//	orb_copy(ORB_ID(actuator_armed), _aekf->_armedSub, &_aekf->_armed);
		}

		//Update Gyro and Accelerometer
		static struct Vector3f lastAngRate;
		static struct Vector3f lastAccel;
		_Bool accel_updated = -1;

		//orb_copy(ORB_ID(sensor_combined), _aekf->_sensor_combined_sub, &_aekf->_sensor_combined);

		static hrt_abstime last_accel;
		static hrt_abstime last_mag;

		//		if (last_accel != _aekf->_sensor_combined.accelerometer_timestamp) {
		accel_updated = 1;

		//		} else {
		accel_updated = -1;
		//		}

		//		last_accel = _aekf->_sensor_combined.accelerometer_timestamp;


		// Copy gyro and accel
		/*		_aekf->_last_sensor_timestamp = _aekf->_sensor_combined.timestamp;
				IMUmsec = _aekf->_sensor_combined.timestamp / 1e3;
				IMUusec = _aekf->_sensor_combined.timestamp;
		 */
		float deltaT;// = (_aekf->_sensor_combined.timestamp - _aekf->_last_run) / 1e6f;

		/* guard against too large deltaT's */
		//	if (!PX4_ISFINITE(deltaT) || deltaT > 1.0f || deltaT < 0.000001f) {
		deltaT = 0.01f;
		//	}

		//	_aekf->_last_run = _aekf->_sensor_combined.timestamp;

		// Always store data, independent of init status
		/* fill in last data set */
		//	_ekf->dtIMU = deltaT;

		int last_gyro_main;// = _aekf->_gyro_main;

		/*if (PX4_ISFINITE(_aekf->_sensor_combined.gyro_rad_s[0]) &&
		  PX4_ISFINITE(_aekf->_sensor_combined.gyro_rad_s[1]) &&
		  PX4_ISFINITE(_aekf->_sensor_combined.gyro_rad_s[2]) &&
		  (_aekf->_sensor_combined.gyro_errcount <= (_aekf->_sensor_combined.gyro1_errcount + GYRO_SWITCH_HYSTERESIS))) {

		  _ekf->angRate.x = _aekf->_sensor_combined.gyro_rad_s[0];
		  _ekf->angRate.y = _aekf->_sensor_combined.gyro_rad_s[1];
		  _ekf->angRate.z = _aekf->_sensor_combined.gyro_rad_s[2];
		  _aekf->_gyro_main = 0;
		  _aekf->_gyro_valid = true;

		  } else if (PX4_ISFINITE(_aekf->_sensor_combined.gyro1_rad_s[0]) &&
		  PX4_ISFINITE(_aekf->_sensor_combined.gyro1_rad_s[1]) &&
		  PX4_ISFINITE(_aekf->_sensor_combined.gyro1_rad_s[2])) {

		  _ekf->angRate.x = _aekf->_sensor_combined.gyro1_rad_s[0];
		  _ekf->angRate.y = _aekf->_sensor_combined.gyro1_rad_s[1];
		  _ekf->angRate.z = _aekf->_sensor_combined.gyro1_rad_s[2];
		  _aekf->_gyro_main = 1;
		  _aekf->_gyro_valid = true;

		  } else {
		  _aekf->_gyro_valid = false;
		  }*/

		/*if (last_gyro_main != _aekf->_gyro_main) {
		  mavlink_and_console_log_emergency(_aekf->_mavlink_fd, "GYRO FAILED! Switched from #%d to %d", last_gyro_main, _aekf->_gyro_main);
		  }*/

		//if (!_aekf->_gyro_valid) {
		/* keep last value if he hit an out of band value */
		//	lastAngRate = _ekf->angRate;

		//} else {
		//	perf_count(_aekf->_perf_gyro);
		//}

		if (accel_updated) {

			int last_accel_main;// = _aekf->_accel_main;

			/* fail over to the 2nd accel if we know the first is down */
			/*			if (_aekf->_sensor_combined.accelerometer_errcount <= (_aekf->_sensor_combined.accelerometer1_errcount + ACCEL_SWITCH_HYSTERESIS)) {
						_ekf->accel.x = _aekf->_sensor_combined.accelerometer_m_s2[0];
						_ekf->accel.y = _aekf->_sensor_combined.accelerometer_m_s2[1];
						_ekf->accel.z = _aekf->_sensor_combined.accelerometer_m_s2[2];
						_aekf->_accel_main = 0;

						} else {
						_ekf->accel.x = _aekf->_sensor_combined.accelerometer1_m_s2[0];
						_ekf->accel.y = _aekf->_sensor_combined.accelerometer1_m_s2[1];
						_ekf->accel.z = _aekf->_sensor_combined.accelerometer1_m_s2[2];
						_aekf->_accel_main = 1;
						}

						if (!_aekf->_accel_valid) {
						lastAccel = _ekf->accel;
						}

						if (last_accel_main != _aekf->_accel_main) {
						mavlink_and_console_log_emergency(_aekf->_mavlink_fd, "ACCEL FAILED! Switched from #%d to %d", last_accel_main, _aekf->_accel_main);
						}

						_aekf->_accel_valid = true;*/
		}

		//_ekf->dAngIMU = 0.5f * (operator_plus(_ekf->angRate , lastAngRate)) * _ekf->dtIMU;
		/*		lastAngRate = _ekf->angRate;
		//_ekf->dVelIMU = 0.5f * (operator_plus(_ekf->accel , lastAccel)) * _ekf->dtIMU;
		lastAccel = _ekf->accel;

		if (last_mag != _aekf->_sensor_combined.magnetometer_timestamp) {
		_aekf->_newDataMag = true;

		} else {
		_aekf->_newDataMag = false;
		}

		last_mag = _aekf->_sensor_combined.magnetometer_timestamp;
		 */
		//PX4_INFO("dang: %8.4f %8.4f dvel: %8.4f %8.4f", _ekf->dAngIMU.x, _ekf->dAngIMU.z, _ekf->dVelIMU.x, _ekf->dVelIMU.z);

		//Update Land Detector
		_Bool newLandData = -1;
		//orb_check(_aekf->_landDetectorSub, &newLandData);

		if (newLandData) {
			//	orb_copy(ORB_ID(vehicle_land_detected), _aekf->_landDetectorSub, &_aekf->_landDetector);
		}

		//Update AirSpeed
		//orb_check(_aekf->_airspeed_sub, &_aekf->_newAdsData);

		//if (_aekf->_newAdsData) {
		//	orb_copy(ORB_ID(airspeed), _aekf->_airspeed_sub, &_aekf->_airspeed);
		//	perf_count(_aekf->_perf_airspeed);

		//	_ekf->VtasMeas =_aekf->_airspeed.true_airspeed_unfiltered_m_s;
		//}


		_Bool gps_update = -1;
		//orb_check(_aekf->_gps_sub, &gps_update);

		if (gps_update) {
			//	orb_copy(ORB_ID(vehicle_gps_position), _aekf->_gps_sub, &_aekf->_gps);
			//	perf_count(_aekf->_perf_gps);

			//We are more strict for our first fix
			float requiredAccuracy;// = _aekf->_parameters.pos_stddev_threshold;

			/*		if (_aekf->_gpsIsGood) {
					requiredAccuracy = _aekf->_parameters.pos_stddev_threshold * 2.0f;
					}

			//Check if the GPS fix is good enough for us to use
			if (_aekf->_gps.fix_type >= 3 && _aekf->_gps.eph < requiredAccuracy && _aekf->_gps.epv < requiredAccuracy) {
			_aekf->_gpsIsGood = true;

			} else {
			_aekf->_gpsIsGood = false;
			}
			 */
			/*			if (_aekf->_gpsIsGood) {

			//Calculate time since last good GPS fix
			const float dtLastGoodGPS = (float)(_aekf->_gps.timestamp_position - _aekf->_previousGPSTimestamp) / 1e6f;

			//Stop dead-reckoning mode
			if (_aekf->_global_pos.dead_reckoning) {
			mavlink_log_info(_aekf->_mavlink_fd, "[ekf] stop dead-reckoning");
			}

			_aekf->_global_pos.dead_reckoning = false;

			//Fetch new GPS data
			_ekf->GPSstatus = _aekf->_gps.fix_type;
			_ekf->velNED[0] = _aekf->_gps.vel_n_m_s;
			_ekf->velNED[1] = _aekf->_gps.vel_e_m_s;
			_ekf->velNED[2] = _aekf->_gps.vel_d_m_s;

			_ekf->gpsLat = radians(_aekf->_gps.lat / (double)1e7);
			_ekf->gpsLon = radians(_aekf->_gps.lon / (double)1e7) - M_PI;
			_ekf->gpsHgt = _aekf->_gps.alt / 1e3f;

			if (_aekf->_previousGPSTimestamp != 0) {
			//Calculate average time between GPS updates
			_ekf->updateDtGpsFilt(_ekf,constrain(dtLastGoodGPS, 0.01f, POS_RESET_THRESHOLD));

			// update LPF
			float filter_step = (dtLastGoodGPS / (rc + dtLastGoodGPS)) * (_ekf->gpsHgt - _aekf->_gps_alt_filt);

			if (PX4_ISFINITE(filter_step)) {
			_aekf->_gps_alt_filt += filter_step;
			}
			}

			//check if we had a GPS outage for a long time
			if (_aekf->_gps_initialized) {

			//Convert from global frame to local frame
			map_projection_project(&_aekf->_pos_ref, (_aekf->_gps.lat / 1.0e7), (_aekf->_gps.lon / 1.0e7), &_ekf->posNE[0], &_ekf->posNE[1]);

			if (dtLastGoodGPS > POS_RESET_THRESHOLD) {
			AttPosEKF_ResetPosition(_ekf);
			AttPosEKF_ResetVelocity(_ekf);

			}
			}

			//PX4_INFO("gps alt: %6.1f, interval: %6.3f", (double)_ekf->gpsHgt, (double)dtGoodGPS);

			// if (_gps.s_variance_m_s > 0.25f && _gps.s_variance_m_s < 100.0f * 100.0f) {
			//	_ekf->vneSigma = sqrtf(_gps.s_variance_m_s);
			// } else {
			//	_ekf->vneSigma = _parameters.velne_noise;
			// }

			// if (_gps.p_variance_m > 0.25f && _gps.p_variance_m < 100.0f * 100.0f) {
			//	_ekf->posNeSigma = sqrtf(_gps.p_variance_m);
			// } else {
			//	_ekf->posNeSigma = _parameters.posne_noise;
			// }

			//PX4_INFO("vel: %8.4f pos: %8.4f", _gps.s_variance_m_s, _gps.p_variance_m);

			_aekf->_previousGPSTimestamp = _aekf->_gps.timestamp_position;

			}*/
		}

		// If it has gone more than POS_RESET_THRESHOLD amount of seconds since we received a GPS update,
		// then something is very wrong with the GPS (possibly a hardware failure or comlink error)
		/*const float dtLastGoodGPS = (float)(_aekf->_gps.timestamp_position - _aekf->_previousGPSTimestamp) / 1e6f;

		  if (dtLastGoodGPS >= POS_RESET_THRESHOLD) {

		  if (_aekf->_global_pos.dead_reckoning) {
		  mavlink_log_info(_aekf->_mavlink_fd, "[ekf] gave up dead-reckoning after long timeout");
		  }

		  _aekf->_gpsIsGood = false;
		  _aekf->_global_pos.dead_reckoning = false;
		  }
		 */
		//If we have no good GPS fix for half a second, then enable dead-reckoning mode while armed (for up to POS_RESET_THRESHOLD seconds)
		/*		else if (dtLastGoodGPS >= 0.5f) {
				if (_aekf->_armed.armed) {
				if (!_aekf->_global_pos.dead_reckoning) {
				mavlink_log_info(_aekf->_mavlink_fd, "[ekf] dead-reckoning enabled");
				}

				_aekf->_global_pos.dead_reckoning = true;

				} else {
				_aekf->_global_pos.dead_reckoning = false;
				}
				}
		 */
		//Update barometer
		//		orb_check(_aekf->_baro_sub, &_aekf->_newHgtData);

		/*		if (_aekf->_newHgtData) {
				static hrt_abstime baro_last = 0;

				orb_copy(ORB_ID(sensor_baro), _aekf->_baro_sub, &_aekf->_baro);

		// init lowpass filters for baro and gps altitude
		float baro_elapsed;

		if (baro_last == 0) {
		baro_elapsed = 0.0f;

		} else {
		baro_elapsed = (_aekf->_baro.timestamp - baro_last) / 1e6f;
		}

		baro_last = _aekf->_baro.timestamp;
		if (!_aekf->_baro_init) {
		_aekf->_baro_init = true;
		_aekf->_baro_alt_filt = _aekf->_baro.altitude;
		}

		_ekf->updateDtHgtFilt(_ekf,constrain(baro_elapsed, 0.001f, 0.1f));

		_ekf->baroHgt = _aekf->_baro.altitude;
		float filter_step = (baro_elapsed / (rc + baro_elapsed)) * (_aekf->_baro.altitude - _aekf->_baro_alt_filt);

		if (PX4_ISFINITE(filter_step)) {
		_aekf->_baro_alt_filt += filter_step;
		}

		perf_count(_aekf->_perf_baro);
		}
		 */
		//Update Magnetometer
		/*		if (_aekf->_newDataMag) {

				_aekf->_mag_valid = true;

				perf_count(_aekf->_perf_mag);

				int last_mag_main = _aekf->_mag_main;

		//struct Vector3f mag0(_aekf->_sensor_combined.magnetometer_ga[0], _aekf->_sensor_combined.magnetometer_ga[1],_aekf->_sensor_combined.magnetometer_ga[2]);

		//struct Vector3f mag1(_aekf->_sensor_combined.magnetometer1_ga[0], _aekf->_sensor_combined.magnetometer1_ga[1],_aekf->_sensor_combined.magnetometer1_ga[2]);

		const unsigned mag_timeout_us = 200000;

		struct mag0 *_mag_p;*/
		/* fail over to the 2nd mag if we know the first is down */
		/*if (hrt_elapsed_time(&_aekf->_sensor_combined.magnetometer_timestamp) < mag_timeout_us &&
		  _aekf->_sensor_combined.magnetometer_errcount <= (_aekf->_sensor_combined.magnetometer1_errcount + MAG_SWITCH_HYSTERESIS) &&
		  _mag_p->length(_mag_p) > 0.1f) {
		  _ekf->magData.x = _aekf->_mag.x;
		  _ekf->magBias.x = 0.000001f; // _mag_offsets.x_offset

		  _ekf->magData.y = _aekf->_mag.y;
		  _ekf->magBias.y = 0.000001f; // _mag_offsets.y_offset

		  _ekf->magData.z = _aekf->_mag.z;
		  _ekf->magBias.z = 0.000001f; // _mag_offsets.y_offset
		  _aekf->_mag_main = 0;

		  } else if (hrt_elapsed_time(&_aekf->_sensor_combined.magnetometer1_timestamp) < mag_timeout_us &&
		  _mag_p->length(_mag_p) > 0.1f) {
		  _ekf->magData.x = _aekf->_mag.x;
		  _ekf->magBias.x = 0.000001f; // _mag_offsets.x_offset

		  _ekf->magData.y = _aekf->_mag.y;
		  _ekf->magBias.y = 0.000001f; // _mag_offsets.y_offset

		  _ekf->magData.z = _aekf->_mag.z;
		  _ekf->magBias.z = 0.000001f; // _mag_offsets.y_offset
		  _aekf->_mag_main = 1;
		  }else {
		  _aekf->_mag_valid = false;
		  }

		  if (last_mag_main != _aekf->_mag_main) {
		  mavlink_and_console_log_emergency(_aekf->_mavlink_fd, "MAG FAILED! Failover from unit %d to unit %d", last_mag_main, _aekf->_mag_main);
		  }*/
	}

	//Update range data
	//orb_check(_aekf->_distance_sub, &_aekf->_newRangeData);

	/*if (_aekf->_newRangeData) {
	  orb_copy(ORB_ID(distance_sensor), _aekf->_distance_sub, &_aekf->_distance);
	  if ((_aekf->_distance.current_distance > _aekf->_distance.min_distance)
	  && (_aekf->_distance.current_distance < _aekf->_distance.max_distance)) {
	  _ekf->rngMea = _aekf->_distance.current_distance;
	  _aekf->_distance_last_valid = _aekf->_distance.timestamp;

	  } else {
	  _aekf->_newRangeData = false;
	  }
	  }*/
	//	}

	int AttitudePositionEstimatorEKF_trip_nan(struct AttitudePositionEstimatorEKF * _aekf)
	{

		int ret = 0;

		// If system is not armed, inject a NaN value into the filter
		//if (_aekf->_armed.armed) {
		//PX4_INFO("ACTUATORS ARMED! NOT TRIPPING SYSTEM");
		ret = 1;

		//} else {

		float nan_val = 0.0f / 0.0f;

		//PX4_INFO("system not armed, tripping state vector with NaN");
		//	_ekf->states[5] = nan_val;
		usleep(100000);

		//PX4_INFO("tripping covariance #1 with NaN");
		//	_ekf->KH[2][2] = nan_val; //  intermediate result used for covariance updates
		usleep(100000);

		//PX4_INFO("tripping covariance #2 with NaN");
		//	_ekf->KHP[5][5] = nan_val; // intermediate result used for covariance updates
		usleep(100000);

		//PX4_INFO("tripping covariance #3 with NaN");
		//	_ekf->P[3][3] = nan_val; // covariance matrix
		usleep(100000);

		//PX4_INFO("tripping Kalman gains with NaN");
		//	_ekf->Kfusion[0] = nan_val; // Kalman gains
		usleep(100000);

		//PX4_INFO("tripping stored states[0] with NaN");
		//	_ekf->storedStates[0][0] = nan_val;
		usleep(100000);

		//PX4_INFO("tripping states[9] with NaN");
		//	_ekf->states[9] = nan_val;
		usleep(100000);

		//PX4_INFO("DONE - FILTER STATE:");
		AttitudePositionEstimatorEKF_print_status(_aekf);
		//}

		return ret;
	}

	bool AttitudePositionEstimatorEKF_task_running(struct AttitudePositionEstimatorEKF* task_running)
	{
		return task_running;
	}


	int ekf_att_pos_estimator_main()//(int argc, char *argv[])
	{
		/*	if (argc < 2) {
		//PX4_ERR("usage: ekf_att_pos_estimator {start|stop|status|logging}");
		return 1;
		}

		if (!strcmp(argv[1], "start")) {

		if (g_estimator != NULL) {
		PX4_ERR("already running");
		return 1;
		}

		//estimator::g_estimator = new AttitudePositionEstimatorEKF();
		g_estimator = (struct AttitudePositionEstimatorEKF*)malloc(sizeof(struct AttitudePositionEstimatorEKF));

		if (g_estimator == NULL) {
		PX4_ERR("alloc failed");
		return 1;
		}

		if (g_estimator->start(_aekf) != 1) {
		free(g_estimator);
		g_estimator = NULL;
		PX4_ERR("start failed");
		return 1;
		}

		// avoid memory fragmentation by not exiting start handler until the task has fully started 
		while (g_estimator == NULL || !g_estimator->task_running(_aekf)) {
		usleep(50000);
		PX4_INFO(".");
		}

		PX4_INFO(" ");

		return 0;
		}

		if (g_estimator == NULL) {
		//PX4_ERR("not running");
		return 1;
		}

		if (!strcmp(argv[1], "stop")) {

		free(g_estimator);
		g_estimator = NULL;
		return 0;
		}

		if (!strcmp(argv[1], "status")) {
		//	PX4_INFO("running");

		g_estimator->print_status(_aekf);

		return 0;
		}

		if (!strcmp(argv[1], "trip")) {
		int ret = g_estimator->trip_nan(_aekf);

		return ret;
		}

		if (!strcmp(argv[1], "logging")) {
		int ret = g_estimator->enable_logging(_aekf,true);

		return ret;
		}

		if (!strcmp(argv[1], "debug")) {
		int debug = strtoul(argv[2], NULL, 10);
		int ret = g_estimator->set_debuglevel(_aekf,debug);

		return ret;
	}
	*/
		//PX4_ERR("unrecognized command");
		return 1;
	}



	void main(int argc, char *argv[])
	{
		_aekf = (struct AttitudePositionEstimatorEKF *)malloc(sizeof(struct AttitudePositionEstimatorEKF)); 
		AttitudePositionEstimatorEKF_task_main(_aekf);
		//ekf_att_pos_estimator_main(argc, argv);
		free(_aekf);
		free(_ekf);
	}

