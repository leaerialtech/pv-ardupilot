#ifdef false

#include "PVSprayManager.h"

#include <AP_Mission/AP_Mission.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

namespace PrecisionVision
{

    PVSprayManager::PVSprayManager()   //AP_Mission &mission, const AP_AHRS& ahrs, const AC_Sprayer &sprayer) : 
      //mission_(mission),
      //ahrs_(ahrs), 
      //sprayer_(sprayer)

       // mission_ = AP_Mission::get_singleton();
       // ahrs_ = AP_AHRS::get_singleton();
        //sprayer_ = AC_SPRAYER:::get_singleton();

    {
        _singleton = this; 
    }

    PVSprayManager *PVSprayManager::_singleton;
    PVSprayManager *PVSprayManager::get_singleton()
    {
        return _singleton;
    }


     void PVSprayManager::disableHeadingSprayChecks(){doHeadingChecks_ = false; }

     void PVSprayManager::enableHeadingSprayChecks(){doHeadingChecks_ = true;}

     bool PVSprayManager::isHeadingSprayChecksEnabled() const {return doHeadingChecks_;}


    //pass in these  variables and perform the proper control on the sprayer 
    void PVSprayManager::disable_sprayer_if_necessary(const PVSprayUpdateInfo info)
    {
        AC_Sprayer* sprayer = AC_Sprayer::get_singleton();
        if(!sprayer){return;}

        //while the first versions of the GCS didn't specify it, shortly after "path spray" was implemented we really needed a mechanism to avoid the normal "heading checks" when
        //spray missions were running in auto mode -- as a path spray cornered the sprayer would shut down briefly where the actual desired behavior was a continuous spray.  (There might still be
        //a momentary blip depending on how wapoint spray-on/off is handled, but at least it won't be a significant delay for hard angles). 

        //To do so, later versions of the GCS set an affirmative "ignoresHeadingChecks" field on the spray_cmd (MAV_CMD_USER_1) -- that way earlier clients behave the same way (sending up a 0 causes it to always check headings)  

         //to maintain backward compability if ignore heading checks is not specifically defined, we do a test for "slow-waypoints"
         //and if it is a slow waypoint (one with a delay > 0) we disable the heading checks (this was the original workaround implementaton for spotspray)
         //in future versions the intention will be that heading checks are entirely determined by the mission design or user commands, not fastwaypoint
         //(particularlly since next versions of ardupilot will support S-Curve and do not have the same fast/slow waypoint concept)
        
        
            bool waitForHeadingChecks = true; //by default, we do heading checks. 
            if( info.is_auto_mode)
            {
                if(!info.is_fastwaypoint){
                    waitForHeadingChecks = false; //holds and stop and spray do not need
                }
            
                //don't spray if we haven't reached a waypoint yet
                if(!info.reached_prev_wpt){
                    sprayer->run(false); 
                    return;
                }
            }
        
            if(waitForHeadingChecks){
                bool result = this->_pv_handle_sprayer_heading_checks(info.wp_bearing, info.current_bearing);
                if(!result){ 
                sprayer->run(false);
                return; 
                }
            }


            //if we get here, do final checks to stop the sprayer if needbe
            this->_pv_turnoff_sprayer_if_missing_heartbeat();


    }


    //returns if heading is in-bounds for spray
    bool PVSprayManager::_pv_handle_sprayer_heading_checks(const int32_t wp_bearing, const int32_t current_heading)
    {
       
        bool waitForHeadingChecks = true; //start false, will set shortly 
        int32_t desiredHeading = wp_bearing / 100; 

        //finally, do heading checks if requested
        if(waitForHeadingChecks)
        {
            int32_t currentHeading = (AP::ahrs().yaw_sensor / 100);
            int32_t diff = abs(desiredHeading - currentHeading);

            if(diff > 360){ diff = diff - 360;}

            bool inBounds = (diff < this->get_heading_interval() || diff > (360.0-this->get_heading_interval()));
            if(!inBounds){
                return false; 
            }
        }
        return true;
    }
    


    void PVSprayManager::_pv_turnoff_sprayer_if_missing_heartbeat()
    {
        AC_Sprayer* sprayer = AC_Sprayer::get_singleton();
        if(!sprayer){return;}
        
        #if HAL_SPRAYER_ENABLED

          const uint32_t now = AP_HAL::millis();
          const uint32_t last_heartbeat_ms = gcs().sysid_myggcs_last_seen_time_ms();

        if (last_heartbeat_ms  == 0) {
            sprayer->run(false);
            return;
        }

        // Check if we have gotten a GCS heartbeat recently (GCS sysid must match SYSID_MYGCS parameter)
        if (now - last_heartbeat_ms  > 2000) {
            // Log event if we are recovering from previous gcs failsafe
            sprayer->run(false);
            return;
        }
       #endif
    }

    float PVSprayManager::get_swath_width() const{

        AC_Sprayer* sp = AC_Sprayer::get_singleton();
        if(sp != nullptr){
            return sp->get_swath_width();
        }
        return 0;
    }

    
    float PVSprayManager::get_heading_interval() const {
  
        AC_Sprayer* sp = AC_Sprayer::get_singleton();
        if(sp != nullptr){
            return sp->get_heading_interval();
        }
        return 0;
    }
}
#endif