/* Generated by CIL v. 1.7.3 */
/* print_CIL_Input is false */

typedef unsigned int size_t;
typedef unsigned long long uint64_t;
typedef _Bool bool;
typedef uint64_t hrt_abstime;
struct vehicle_attitude_s {
   float q[22] ;
   _Bool q_valid ;
   _Bool R_valid ;
   float timestamp ;
   int roll ;
   int pitch ;
   int yaw ;
   float rollspeed ;
   float pitchspeed ;
   float yawspeed ;
   float rate_offsets[3] ;
};
struct vehicle_global_position_s {
   int timestamp ;
   int time_utc_usec ;
   float lat ;
   float lon ;
   float alt ;
   double yaw ;
   double eph ;
   double epv ;
   _Bool v_xy_valid ;
   _Bool v_z_valid ;
   float vel_n ;
   float vel_e ;
   float vel_d ;
   _Bool terrain_alt_valid ;
   float terrain_alt ;
   _Bool dead_reckoning ;
};
struct vehicle_local_position_s {
   int timestamp ;
   float x ;
   float y ;
   float z ;
   float vx ;
   float vy ;
   float vz ;
   float yaw ;
   float alt ;
   _Bool xy_valid ;
   _Bool z_valid ;
   _Bool v_xy_valid ;
   _Bool v_z_valid ;
   _Bool xy_global ;
   _Bool z_global ;
   int ref_timestamp ;
   float ref_lat ;
   float ref_lon ;
   float ref_alt ;
   float dist_bottom ;
   float dist_bottom_rate ;
   int surface_bottom_timestamp ;
   _Bool dist_bottom_valid ;
   float eph ;
   float epv ;
};
struct wind_estimate_s {
   int timestamp ;
   float windspeed_north ;
   float windspeed_east ;
   float covariance_north ;
   float covariance_east ;
};
struct sensor_combined_s {
   float gyro_rad_s[3] ;
   float gyro_rad1_s[3] ;
   float gyro1_rad_s[3] ;
   float accelerometer_m_s2[3] ;
   float accelerometer1_m_s2[3] ;
   float magnetometer_ga[3] ;
   int gyro_errcount ;
   int gyro1_errcount ;
   int gyro2_errcount ;
   int accelerometer_errcount ;
   int accelerometer1_errcount ;
   int accelerometer2_errcount ;
   int magnetometer_errcount ;
   int magnetometer1_errcount ;
   int magnetometer2_errcount ;
   int timestamp ;
   int accelerometer_timestamp ;
   int magnetometer_timestamp ;
   int magnetometer1_timestamp ;
};
struct map_projection_reference_s {
   double lat_rad ;
   double lon_rad ;
   double sin_lat ;
   double cos_lat ;
   _Bool init_done ;
   int timestamp ;
};
struct LowPassFilter2p {
   void (*constructor)(struct LowPassFilter2p * , float sample_freq , float cutoff_freq ) ;
   void (*set_cutoff_frequency)(struct LowPassFilter2p * , float sample_freq , float cutoff_freq ) ;
   float (*apply)(struct LowPassFilter2p * , float sample ) ;
   float (*get_cutoff_freq)(struct LowPassFilter2p * ) ;
   float (*reset)(struct LowPassFilter2p * , float sample ) ;
   float _cutoff_freq ;
   float _a1 ;
   float _a2 ;
   float _b0 ;
   float _b1 ;
   float _b2 ;
   float _delay_element_1 ;
   float _delay_element_2 ;
};
struct __anonstruct__parameters_1 {
   float vel_delay_ms ;
   float pos_delay_ms ;
   float height_delay_ms ;
   float mag_delay_ms ;
   float tas_delay_ms ;
   float velne_noise ;
   float veld_noise ;
   float posne_noise ;
   float posd_noise ;
   float mag_noise ;
   float gyro_pnoise ;
   float acc_pnoise ;
   float gbias_pnoise ;
   float abias_pnoise ;
   float mage_pnoise ;
   float magb_pnoise ;
   float eas_noise ;
   float pos_stddev_threshold ;
};
struct __anonstruct__parameter_handles_2 {
   int *vel_delay_ms ;
   int *pos_delay_ms ;
   int *height_delay_ms ;
   int *mag_delay_ms ;
   int *tas_delay_ms ;
   int *velne_noise ;
   int *veld_noise ;
   int *posne_noise ;
   int *posd_noise ;
   int *mag_noise ;
   int *gyro_pnoise ;
   int *acc_pnoise ;
   int *gbias_pnoise ;
   int *abias_pnoise ;
   int *mage_pnoise ;
   int *magb_pnoise ;
   int *eas_noise ;
   int *pos_stddev_threshold ;
};
struct AttitudePositionEstimatorEKF {
   void (*constructor)(struct AttitudePositionEstimatorEKF * ) ;
   void (*destructor)(struct AttitudePositionEstimatorEKF * ) ;
   int (*start)(struct AttitudePositionEstimatorEKF * ) ;
   bool (*task_running)(struct AttitudePositionEstimatorEKF * ) ;
   void (*print_status)(struct AttitudePositionEstimatorEKF * ) ;
   int (*trip_nan)(struct AttitudePositionEstimatorEKF * ) ;
   int (*enable_logging)(struct AttitudePositionEstimatorEKF * , bool enable ) ;
   int (*set_debuglevel)(struct AttitudePositionEstimatorEKF * , unsigned int debug ) ;
   bool _task_should_exit ;
   bool _task_running ;
   int _estimator_task ;
   struct sensor_combined_s _sensor_combined ;
   struct map_projection_reference_s _pos_ref ;
   float _filter_ref_offset ;
   float _baro_gps_offset ;
   hrt_abstime _last_debug_print ;
   float _gps_alt_filt ;
   float _baro_alt_filt ;
   float _covariancePredictionDt ;
   bool _gpsIsGood ;
   uint64_t _previousGPSTimestamp ;
   bool _baro_init ;
   bool _gps_initialized ;
   hrt_abstime _filter_start_time ;
   hrt_abstime _last_sensor_timestamp ;
   hrt_abstime _last_run ;
   hrt_abstime _distance_last_valid ;
   bool _gyro_valid ;
   bool _accel_valid ;
   bool _mag_valid ;
   int _gyro_main ;
   int _accel_main ;
   int _mag_main ;
   bool _ekf_logging ;
   struct vehicle_attitude_s _att ;
   struct vehicle_local_position_s _local_pos ;
   struct vehicle_global_position_s _global_pos ;
   struct wind_estimate_s _wind ;
   bool _newHgtData ;
   bool _newAdsData ;
   bool _newDataMag ;
   bool _newRangeData ;
   int _mavlink_fd ;
   struct __anonstruct__parameters_1 _parameters ;
   struct __anonstruct__parameter_handles_2 _parameter_handles ;
   struct LowPassFilter2p _LP_att_P ;
   struct LowPassFilter2p _LP_att_Q ;
   struct LowPassFilter2p _LP_att_R ;
   int (*parameters_update)(struct AttitudePositionEstimatorEKF * ) ;
   void (*control_update)(struct AttitudePositionEstimatorEKF * ) ;
   void (*vehicle_status_poll)(struct AttitudePositionEstimatorEKF * ) ;
   void (*task_main_trampoline)(struct AttitudePositionEstimatorEKF * , int argc , char **argv ) ;
   void (*task_main)(struct AttitudePositionEstimatorEKF * ) ;
   int (*check_filter_state)(struct AttitudePositionEstimatorEKF * ) ;
   void (*publishAttitude)(struct AttitudePositionEstimatorEKF * ) ;
   void (*publishLocalPosition)(struct AttitudePositionEstimatorEKF * ) ;
   void (*publishGlobalPosition)(struct AttitudePositionEstimatorEKF * ) ;
   void (*publishWindEstimate)(struct AttitudePositionEstimatorEKF * ) ;
   void (*updateSensorFusion)(struct AttitudePositionEstimatorEKF * , bool const   fuseGPS , bool const   fuseMag , bool const   fuseRangeSensor , bool const   fuseBaro , bool const   fuseAirSpeed ) ;
   void (*initializeGPS)(struct AttitudePositionEstimatorEKF * ) ;
   void (*initReferencePosition)(struct AttitudePositionEstimatorEKF * , hrt_abstime timestamp , double lat , double lon , float gps_alt , float baro_alt ) ;
   void (*pollData)(struct AttitudePositionEstimatorEKF * ) ;
};
struct AttPosEKF;
struct Vector3f;
struct estimator_status_s {
   int nan_flags ;
   int health_flags ;
   int timeout_flags ;
   float states[32] ;
   int n_states ;
   float covariances[28] ;
   float timestamp ;
};
struct ekf_status_report;
struct pollfd;
struct parameter_update_s;
float rc  =    10.0f;
uint64_t FILTER_INIT_DELAY  =    1000000;
float POS_RESET_THRESHOLD  =    5.0f;
unsigned int MAG_SWITCH_HYSTERESIS  =    10;
unsigned int GYRO_SWITCH_HYSTERESIS  =    5;
unsigned int ACCEL_SWITCH_HYSTERESIS  =    5;
int const   ERROR  =    -1;
struct AttitudePositionEstimatorEKF *_aekf  ;
struct LowPassFilter2p *_lpf  ;
struct AttPosEKF *_ekf  ;
struct Vector3f *_vec3f  ;
void usleep(int usec ) 
{ 
  int sleep_usec ;

  {
  sleep_usec = usec;
  return;
}
}
int orb_publish(char *meta , int st_size ) 
{ 
  char *topic_name ;
  int topic_size ;

  {
  topic_name = meta;
  topic_size = st_size;
  return (1);
}
}
void LowPassFilter2p_constructor(struct LowPassFilter2p *_lpf___0 , float sample_freq , float cutoff_freq ) 
{ 


  {
  return;
}
}
void LowPassFilter2p_get_cutoff_freq(struct LowPassFilter2p *get_cutoff_freq ) 
{ 


  {
  return;
}
}
void LowPassFilter2p_set_cutoff_frequency(struct LowPassFilter2p *_lpf___0 , float sample_freq , float cutoff_freq ) 
{ 


  {
  return;
}
}
float LowPassFilter2p_apply(struct LowPassFilter2p *_lpf___0 , float sample ) 
{ 
  float output ;

  {
  return (output);
}
}
void LowPassFilter2p_reset(struct LowPassFilter2p *_lpf___0 , float sample ) 
{ 


  {
  return;
}
}
struct AttitudePositionEstimatorEKF *g_estimator  ;
extern int ( /* missing proto */  parameters_update)() ;
void AttitudePositionEstimatorEKF_constructor(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 


  {
  parameters_update();
  return;
}
}
void AttitudePositionEstimatorEKF_destroy(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  unsigned int i ;
  int usec3 ;
  int sleep_usec4 ;

  {
  {
  i = 0;
  usec3 = 20000;
  {
  sleep_usec4 = usec3;
  goto Lret_usleep;
  }
  Lret_usleep: /* CIL Label */ ;
  i ++;
  }
  return;
}
}
int AttitudePositionEstimatorEKF_enable_logging(struct AttitudePositionEstimatorEKF *_aekf___0 , bool logging ) 
{ 


  {
  return (0);
}
}
int AttitudePositionEstimatorEKF_parameters_update(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 


  {
  return (1);
}
}
void AttitudePositionEstimatorEKF_vehicle_status_poll(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  _Bool vstatus_updated ;

  {
  vstatus_updated = -1;
  return;
}
}
void AttitudePositionEstimatorEKF_control_update(struct AttitudePositionEstimatorEKF *control_update ) 
{ 


  {
  return;
}
}
extern int ( /* missing proto */  nondet_int)() ;
extern int ( /* missing proto */  memset)() ;
extern int ( /* missing proto */  AttPosEKF_GetFilterState)() ;
int AttitudePositionEstimatorEKF_check_filter_state(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  struct ekf_status_report ekf_report ;
  int check ;
  int checkrand ;
  char const   *feedback[8] ;
  unsigned int warn_index ;
  unsigned int max_warn_index ;
  struct estimator_status_s rep ;
  int _estimator_status_pub_rand ;
  char *meta12 ;
  int st_size13 ;
  char *topic_name14 ;
  int topic_size15 ;

  {
  check = 0;
  checkrand = 0;
  checkrand = nondet_int();
  if (checkrand == 1) {
    check = 1;
  }
  feedback[0] = 0;
  feedback[1] = "NaN in states, resetting";
  feedback[2] = "stale sensor data, resetting";
  feedback[3] = "got initial position lock";
  feedback[4] = "excessive gyro offsets";
  feedback[5] = "velocity diverted, check accel config";
  feedback[6] = "excessive covariances";
  feedback[7] = "unknown condition, resetting";
  if (check) {
    warn_index = (unsigned int )check;
    max_warn_index = sizeof(feedback) / sizeof(feedback[0]);
    if (max_warn_index < warn_index) {
      warn_index = max_warn_index;
    }
  }
  {
  memset(& rep, 0, sizeof(rep));
  AttPosEKF_GetFilterState(_ekf, & ekf_report);
  _estimator_status_pub_rand = 0;
  _estimator_status_pub_rand = nondet_int();
  meta12 = "estimator_status";
  st_size13 = sizeof(rep);
  {
  topic_name14 = meta12;
  topic_size15 = st_size13;
  goto Lret_orb_publish;
  }
  Lret_orb_publish: /* CIL Label */ ;
  }
  return (check);
}
}
void AttitudePositionEstimatorEKF_task_main_trampoline(struct AttitudePositionEstimatorEKF *_aekf___0 , int argc , char **argv ) 
{ 


  {
  return;
}
}
extern int ( /* missing proto */  malloc)() ;
extern int ( /* missing proto */  AttPosEKF_constructor)() ;
extern int ( /* missing proto */  PX4_ERR)() ;
extern int ( /* missing proto */  rand)() ;
extern int ( /* missing proto */  px4_poll)() ;
void AttitudePositionEstimatorEKF_pollData(struct AttitudePositionEstimatorEKF *_aekf___0 ) ;
void AttitudePositionEstimatorEKF_publishAttitude(struct AttitudePositionEstimatorEKF *_aekf___0 ) ;
void AttitudePositionEstimatorEKF_publishLocalPosition(struct AttitudePositionEstimatorEKF *_aekf___0 ) ;
void AttitudePositionEstimatorEKF_publishGlobalPosition(struct AttitudePositionEstimatorEKF *_aekf___0 ) ;
void AttitudePositionEstimatorEKF_publishWindEstimate(struct AttitudePositionEstimatorEKF *_aekf___0 ) ;
void AttitudePositionEstimatorEKF_task_main(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  struct pollfd fds[2] ;
  int idx ;
  int loopcnt ;
  int tmp___1 ;
  int pret ;
  int tmp___2 ;
  _Bool prev_hil ;
  float initVelNED[3] ;
  int check ;
  int tmp___3 ;
  int usec15 ;
  int sleep_usec16 ;

  {
  AttitudePositionEstimatorEKF_constructor(_aekf___0);
  _ekf = (struct AttPosEKF *)malloc(sizeof(struct AttPosEKF ));
  AttPosEKF_constructor(_ekf);
  _lpf = (struct LowPassFilter2p *)malloc(sizeof(struct LowPassFilter2p ));
  if (! _ekf) {
    PX4_ERR("OUT OF MEM!");
    return;
  }
  parameters_update();
  idx = 0;
  idx = 0;
  while (idx < 5) {
    loopcnt = 0;
    tmp___1 = rand();
    loopcnt = tmp___1 % 2;
    if (loopcnt == 1) {
      break;
    }
    {
    tmp___2 = px4_poll(& fds[0], sizeof(fds) / sizeof(fds[0]), 100);
    pret = tmp___2;
    parameters_update();
    prev_hil = -1;
    AttitudePositionEstimatorEKF_vehicle_status_poll(_aekf___0);
    usec15 = 60000;
    {
    sleep_usec16 = usec15;
    goto Lret_usleep;
    }
    Lret_usleep: /* CIL Label */ ;
    AttitudePositionEstimatorEKF_pollData(_aekf___0);
    initVelNED[0] = 0.0f;
    initVelNED[1] = 0.0f;
    initVelNED[2] = 0.0f;
    tmp___3 = AttitudePositionEstimatorEKF_check_filter_state(_aekf___0);
    check = tmp___3;
    }
    if (check) {
      goto __Cont;
    }
    AttitudePositionEstimatorEKF_publishAttitude(_aekf___0);
    AttitudePositionEstimatorEKF_publishLocalPosition(_aekf___0);
    AttitudePositionEstimatorEKF_publishGlobalPosition(_aekf___0);
    AttitudePositionEstimatorEKF_publishWindEstimate(_aekf___0);
    __Cont: /* CIL Label */ 
    idx ++;
  }
  return;
}
}
void AttitudePositionEstimatorEKF_initReferencePosition(struct AttitudePositionEstimatorEKF *_aekf___0 , hrt_abstime timestamp , double lat , double lon , float gps_alt , float baro_alt ) 
{ 


  {
  return;
}
}
extern int ( /* missing proto */  radians)() ;
extern int ( /* missing proto */  get_mag_declination)() ;
void AttitudePositionEstimatorEKF_initializeGPS(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  double lat ;
  double lon ;
  float declination ;
  int tmp ;
  float tmp___0 ;

  {
  tmp = get_mag_declination(lat, lon);
  tmp___0 = (float )radians(tmp);
  declination = tmp___0;
  return;
}
}
void AttitudePositionEstimatorEKF_publishAttitude(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  int i ;
  int j ;
  int _att_pub_rand ;
  char *meta6 ;
  int st_size7 ;
  char *topic_name8 ;
  int topic_size9 ;
  char *meta10 ;
  int st_size11 ;
  char *topic_name12 ;
  int topic_size13 ;
  char *meta14 ;
  int st_size15 ;
  char *topic_name16 ;
  int topic_size17 ;

  {
  i = 0;
  while (i < 3) {
    j = 0;
    while (j < 3) {
      j ++;
    }
    i ++;
  }
  {
  _att_pub_rand = 0;
  _att_pub_rand = nondet_int();
  meta6 = "vehicle_attitude";
  st_size7 = sizeof(_aekf___0->_att);
  {
  topic_name8 = meta6;
  topic_size9 = st_size7;
  goto Lret_orb_publish;
  }
  Lret_orb_publish: /* CIL Label */ ;
  meta10 = "vehicle_attitude";
  st_size11 = sizeof(_aekf___0->_att);
  {
  topic_name12 = meta10;
  topic_size13 = st_size11;
  goto Lret_orb_publish___0;
  }
  Lret_orb_publish___0: /* CIL Label */ ;
  meta14 = "estimator_status";
  st_size15 = sizeof(_aekf___0->_att);
  {
  topic_name16 = meta14;
  topic_size17 = st_size15;
  goto Lret_orb_publish___1;
  }
  Lret_orb_publish___1: /* CIL Label */ ;
  }
  return;
}
}
void AttitudePositionEstimatorEKF_publishLocalPosition(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  int _local_pos_pub_rand ;
  char *meta3 ;
  int st_size4 ;
  char *topic_name5 ;
  int topic_size6 ;

  {
  {
  _local_pos_pub_rand = 0;
  _local_pos_pub_rand = nondet_int();
  meta3 = "vehicle_local_position";
  st_size4 = sizeof(_aekf___0->_local_pos);
  {
  topic_name5 = meta3;
  topic_size6 = st_size4;
  goto Lret_orb_publish;
  }
  Lret_orb_publish: /* CIL Label */ ;
  }
  return;
}
}
void AttitudePositionEstimatorEKF_publishGlobalPosition(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  int _global_pos_pub_rand ;
  char *meta3 ;
  int st_size4 ;
  char *topic_name5 ;
  int topic_size6 ;

  {
  {
  _global_pos_pub_rand = 0;
  _global_pos_pub_rand = nondet_int();
  meta3 = "vehicle_global_position";
  st_size4 = sizeof(_aekf___0->_global_pos);
  {
  topic_name5 = meta3;
  topic_size6 = st_size4;
  goto Lret_orb_publish;
  }
  Lret_orb_publish: /* CIL Label */ ;
  }
  return;
}
}
void AttitudePositionEstimatorEKF_publishWindEstimate(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  int _wind_pub_rand ;
  char *meta3 ;
  int st_size4 ;
  char *topic_name5 ;
  int topic_size6 ;

  {
  {
  _wind_pub_rand = 0;
  _wind_pub_rand = nondet_int();
  _wind_pub_rand = 1;
  meta3 = "wind_estimate";
  st_size4 = sizeof(_aekf___0->_wind);
  {
  topic_name5 = meta3;
  topic_size6 = st_size4;
  goto Lret_orb_publish;
  }
  Lret_orb_publish: /* CIL Label */ ;
  }
  return;
}
}
void AttitudePositionEstimatorEKF_updateSensorFusion(struct AttitudePositionEstimatorEKF *_aekf___0 , bool const   fuseGPS , bool const   fuseMag , bool const   fuseRangeSensor , bool const   fuseBaro , bool const   fuseAirSpeed ) 
{ 


  {
  return;
}
}
int AttitudePositionEstimatorEKF_start(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 


  {
  return (1);
}
}
void AttitudePositionEstimatorEKF_print_status(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 


  {
  return;
}
}
void AttitudePositionEstimatorEKF_pollData(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  _Bool armedUpdate ;
  _Bool accel_updated ;
  float deltaT ;
  _Bool newLandData ;
  _Bool gps_update ;

  {
  armedUpdate = -1;
  accel_updated = -1;
  accel_updated = 1;
  accel_updated = -1;
  deltaT = 0.01f;
  newLandData = -1;
  gps_update = -1;
  return;
}
}
int AttitudePositionEstimatorEKF_trip_nan(struct AttitudePositionEstimatorEKF *_aekf___0 ) 
{ 
  int ret ;
  float nan_val ;
  int usec4 ;
  int sleep_usec5 ;
  int usec6 ;
  int sleep_usec7 ;
  int usec8 ;
  int sleep_usec9 ;
  int usec10 ;
  int sleep_usec11 ;
  int usec12 ;
  int sleep_usec13 ;
  int usec14 ;
  int sleep_usec15 ;
  int usec16 ;
  int sleep_usec17 ;

  {
  {
  ret = 0;
  ret = 1;
  nan_val = 0.0f / 0.0f;
  usec4 = 100000;
  {
  sleep_usec5 = usec4;
  goto Lret_usleep;
  }
  Lret_usleep: /* CIL Label */ ;
  usec6 = 100000;
  {
  sleep_usec7 = usec6;
  goto Lret_usleep___0;
  }
  Lret_usleep___0: /* CIL Label */ ;
  usec8 = 100000;
  {
  sleep_usec9 = usec8;
  goto Lret_usleep___1;
  }
  Lret_usleep___1: /* CIL Label */ ;
  usec10 = 100000;
  {
  sleep_usec11 = usec10;
  goto Lret_usleep___2;
  }
  Lret_usleep___2: /* CIL Label */ ;
  usec12 = 100000;
  {
  sleep_usec13 = usec12;
  goto Lret_usleep___3;
  }
  Lret_usleep___3: /* CIL Label */ ;
  usec14 = 100000;
  {
  sleep_usec15 = usec14;
  goto Lret_usleep___4;
  }
  Lret_usleep___4: /* CIL Label */ ;
  usec16 = 100000;
  {
  sleep_usec17 = usec16;
  goto Lret_usleep___5;
  }
  Lret_usleep___5: /* CIL Label */ ;
  AttitudePositionEstimatorEKF_print_status(_aekf___0);
  }
  return (ret);
}
}
bool AttitudePositionEstimatorEKF_task_running(struct AttitudePositionEstimatorEKF *task_running ) 
{ 


  {
  return (task_running);
}
}
int ekf_att_pos_estimator_main(void) 
{ 


  {
  return (1);
}
}
extern int ( /* missing proto */  free)() ;
void main(int argc , char **argv ) 
{ 


  {
  _aekf = (struct AttitudePositionEstimatorEKF *)malloc(sizeof(struct AttitudePositionEstimatorEKF ));
  AttitudePositionEstimatorEKF_task_main(_aekf);
  free(_aekf);
  free(_ekf);
  return;
}
}
