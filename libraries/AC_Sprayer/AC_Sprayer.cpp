/////////////////////////////////////////////////////////////////
//Modified by Leading Edge Aerial Technologies, LLC. (Feb 2021)//
/////////////////////////////////////////////////////////////////

#include "AC_Sprayer.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

#include "../ArduCopter/Copter.h"

extern const AP_HAL::HAL& hal;
extern Copter copter;


static bool firstRun = true; 
// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("CONFIG", 0, AC_Sprayer, _config, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when traveling 1m/s expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standardf
    AP_GROUPINFO("PUMP_RATE",   1, AC_Sprayer, _pump_pct_1ms, AC_SPRAYER_DEFAULT_PUMP_RATE),

    // @Param: SPINNER
    // @DisplayName: Spinner rotation speed
    // @Description: Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPINNER",     2, AC_Sprayer, _spinner_pwm, AC_SPRAYER_DEFAULT_SPINNER_PWM),

    // @Param: SPEED_MIN
    // @DisplayName: Speed minimum
    // @Description: Speed minimum at which we will begin spraying
    // @Units: cm/s
    // @Range: 0 1000fWD_
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN",   3, AC_Sprayer, _speed_min, AC_SPRAYER_DEFAULT_SPEED_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),





//PrecisionVision: 
    // @Param: SWATH_WIDTH
    // @DisplayName: Swath width in Meters
    // @Description: The distance the sprayer-boom / rig is set to produce its output to cover
    // @Units: m
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("SWATH_WD",   5, AC_Sprayer, _swath_width, AC_SPRAYER_DEFAULT_SWATH_WD),


    // @Param: SPRAY_PWM_DEF
    // @DisplayName: stuff
    // @Description: stuff
    // @Units: m
    // @Range: 0 100
    // @User: Standard
    
    AP_GROUPINFO("MTR_DEF",   6, AC_Sprayer, _spray_motor_pwm_default, 1),
    AP_GROUPINFO("MTR_MIN",   7, AC_Sprayer, _spray_motor_pwm_range_min, 0),
    AP_GROUPINFO("MTR_MAX",   8, AC_Sprayer, _spray_motor_pwm_range_max, 0),
    AP_GROUPINFO("MTR_DES",   9, AC_Sprayer, _spray_motor_pwm_desired, 0),

    AP_GROUPINFO("DR_DEF",   10, AC_Sprayer, _spray_door_pwm_default, 0),
    AP_GROUPINFO("DR_MIN",   11, AC_Sprayer, _spray_door_pwm_range_min, 0),
    AP_GROUPINFO("DR_MAX",   12, AC_Sprayer, _spray_door_pwm_range_max, 0),
    AP_GROUPINFO("DR_DES",   13, AC_Sprayer, _spray_door_pwm_desired, 0),


    AP_GROUPINFO("HD_INT", 14, AC_Sprayer, _heading_interval, 15),

    AP_GROUPINFO("MTR2_DEF", 15, AC_Sprayer, _spray_motor_pwm_default2, 1),
    AP_GROUPINFO("MTR2_MIN", 16, AC_Sprayer, _spray_motor_pwm_range_min2, 0),
    AP_GROUPINFO("MTR2_MAX", 17, AC_Sprayer, _spray_motor_pwm_range_max2, 0),
    AP_GROUPINFO("MTR2_DES", 18, AC_Sprayer, _spray_motor_pwm_desired2, 0),



    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many sprayers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }

    // To-Do: ensure that the pump and spinner servo channels are enabled
}

/*
 * Get the AP_Sprayer singleton
 */
AC_Sprayer *AC_Sprayer::_singleton;
AC_Sprayer *AC_Sprayer::get_singleton()
{
    return _singleton;
}

void AC_Sprayer::run(const bool true_false, const bool ignore_HeadingCheck)
{
    _flags.ignore_heading_check = ignore_HeadingCheck ? 1 : 0;

    // return immediately if no change
    if(firstRun && _config > 0){
        firstRun = false; 
        stop_spraying();
        return; 
   }else if (true_false == _flags.running) {
        return;
    }

    // set flag indicate whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled
    _flags.running = true_false && _config > 0;

    // turn off the pump and spinner servos if necessary
    if (!_flags.running) {
        stop_spraying();
    }

    //gcs().send_text(MAV_SEVERITY_ALERT, (_flags.running ? "Spray on" : "Spray is off"));
    
}

void AC_Sprayer::stop_spraying()
{
   // if(!_flags.spraying){return;}

   // if(firstSprayOnForEscCal){
    //    firstSprayOnForEscCal = false;  
   // }
   
     SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump, _spray_motor_pwm_default);

     
     if(_config == 2)
     {

        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spray_door_pwm_default);    
     }else if(_config == 3){


        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spray_motor_pwm_default2);
     }
    



//not sure what this is about

   
    _flags.spraying = false;
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void AC_Sprayer::update()
{
    // exit immediately if we are disabled or shouldn't be running
    if (!_config || !running()) {
        run(false,false);
        return;
    }

    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump)) {
        return;
    }


    /*
    // get horizontal velocity
    Vector3f velocity;
    if (!AP::ahrs().get_velocity_NED(velocity)) {
        // treat unknown velocity as zero which should lead to pump stopping
        // velocity will already be zero but this avoids a coverity warning
        velocity.zero();
    }
    float ground_speed = norm(velocity.x * 100.0f, velocity.y * 100.0f);
    */

    // get the current time
    const uint32_t now = AP_HAL::millis();
    if (copter.failsafe.last_heartbeat_ms  == 0) {
        run(false, false);
        return;
    }
    // Check if we have gotten a GCS heartbeat recently (GCS sysid must match SYSID_MYGCS parameter)
    if (now - copter.failsafe.last_heartbeat_ms  > 2000) {
        // Log event if we are recovering from previous gcs failsafe
        run(false,false);
        return;
    }

    bool should_be_spraying = _flags.spraying;

    bool waitForHeadingChange = !_flags.ignore_heading_check;  //might also include "guided" modes in future? 
    float desiredHeading = 0.0;
    int32_t currentHeading = (AP::ahrs().yaw_sensor / 100);
 
    if(!_flags.ignore_heading_check){ 
        //to maintain backward compability if ignore waypoints is not specifically defined, we do a test for "slow-waypoints"
        //and if it is a slow waypoint (one with a delay > 0) we disable the heading checks (this was the original workaround implementaton for spotspray)
        //in future versions the intention will be that heading checks are entirely determined by the mission design or user commands, not fastwaypoint
        //(particularlly since next versions of ardupilot will support S-Curve and do not have the same fast/slow waypoint concept)
        
        if(copter.control_mode == Mode::Number::AUTO){
            waitForHeadingChange = true;
            if(!copter.wp_nav->is_fast_waypoint()){
                waitForHeadingChange = false; //holds and stop and spray do not need
            }
            desiredHeading = copter.mode_auto.wp_bearing() / 100;
     
        }
    }

    int32_t diff = abs(desiredHeading - currentHeading);
    if(diff > 360){ diff = diff - 360;}
  

    // check our speed vs the minimum
    //if (ground_speed >= _speed_min ) {

        // if we are not already spraying
        if (!_flags.spraying) {
            // set the timer if this is the first time we've surpassed the min speed
            if (_speed_over_min_time == 0) {
                _speed_over_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) {

                    if(!waitForHeadingChange){
                        should_be_spraying = true;
                    }else{
                        should_be_spraying = (diff < _heading_interval || diff > (360.0-_heading_interval));
                    }
                
                    _speed_over_min_time = 0;
                }
            }
        }else{
              if(waitForHeadingChange){
                  should_be_spraying = (diff < _heading_interval || diff > (360.0-_heading_interval));
               }
        }
        // reset the speed under timer
        _speed_under_min_time = 0;
    
    /*
    } else {
        // we are under the min speed.
        if (_flags.spraying) {
            // set the timer if this is the first time we've dropped below the min speed
            if (_speed_under_min_time == 0) {
                _speed_under_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) {
                    should_be_spraying = false;
                    _speed_under_min_time = 0;
                }
            }
        }
        // reset the speed over timer
        _speed_over_min_time = 0;
    }
    */

    // if testing pump output speed as if traveling at 1m/s
    if (_flags.testing) {
      //  ground_speed = 100.0f;
        should_be_spraying = true;
    }


    if(copter.control_mode == Mode::Number::AUTO){
        if(!copter.wp_nav->reached_prev_wpt()){
            should_be_spraying = false; 
        }
    }
    

    // if spraying or testing update the pump servo position
    if (should_be_spraying) {
        //float pos = ground_speed * _pump_pct_1ms;
        //pos = MAX(pos, 100 *_pump_min_pct); // ensure min pump speed
        //pos = MIN(pos,10000); // clamp to range   
        //SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, pos, 0, 10000);
      
       // SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spinner_pwm);
      //PrecisionVision is now sending up the desired PWMs directly in a custom parameter, ignoring the original ardupilot scaling 

       /// SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, _spray_motor_pwm_desired, 0, 10000);
      
    //  if(firstSprayOnForEscCal){

     //   SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump, _spray_motor_pwm_range_max);
    //    SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spray_door_pwm_range_max);

    //  }else{


        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump, _spray_motor_pwm_desired);

        if(_config == 2){
            SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spray_door_pwm_desired);
        }else if (_config == 3){

            SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spray_motor_pwm_desired2);
       }     
     // }

        _flags.spraying = true;
    } else {
        stop_spraying();
    }
}

namespace AP {

AC_Sprayer *sprayer()
{
    return AC_Sprayer::get_singleton();
}

};
