#ifndef PVSPRAY_MANAGER_xx_H
#define PVSPRAY_MANAGER_xx_H

#ifdef false
#include <AP_Mission/AP_Mission.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Sprayer/AC_Sprayer.h>


namespace PrecisionVision
{

    //PrecisionVision: 
    #define AC_SPRAYER_DEFAULT_SWATH_WD       0
    /////

 

     class PVSprayManager
    {        
        public: 
        //all the info important for us to make spray decisions with 
            typedef struct PVSprayUpdateInfo
            {
                bool is_auto_mode;
                int32_t wp_bearing; //the desired wp bearing e.g. from mode_auto ->get_wp_bearing()
                int32_t current_bearing;  //the current bearing of the drone  e.g. copter->get_heading_interval()
                float ground_speed;
                bool is_fastwaypoint;
                bool reached_prev_wpt;

            } PVSprayUpdateInfo;

    
            PVSprayManager();
            
            /* Do not allow copies */
            CLASS_NO_COPY(PVSprayManager);

            static PVSprayManager *get_singleton();
            static PVSprayManager *_singleton;

            //methods
            void disableHeadingSprayChecks();
            void enableHeadingSprayChecks();
            bool isHeadingSprayChecksEnabled() const; 

            //main update routine -- takes these inputs and controls the sprayer 
            void disable_sprayer_if_necessary(const PVSprayUpdateInfo info);
            

            //pull these from the existing AC_Spray parameters (legacy design0 )
            
            float get_swath_width() const;;// const { return _swath_width;} 
            float get_heading_interval() const;// const { return _heading_interval;}

            void set_desired_heading(int32_t hd){_desired_heading = hd;}
            //void set_wait_for_heading_checks(bool tf){_wait_for_heading_checks = tf;}

        private: 

         	//PrecisionVision:
           // AP_Float _swath_width;            //distance in meters that the boom/rig is set to output (we use this to output to observers, possibly do calculations in future) 
           // AP_Float _heading_interval; //plus or minus this amount for heading checks if active 
            int32_t _desired_heading;
            ///


        bool doHeadingChecks_= true; 

       bool _pv_handle_sprayer_heading_checks(const int32_t wp_bearing, const int32_t current_heading);
        void _pv_turnoff_sprayer_if_missing_heartbeat();
       void _pv_handle_path_spray();

       
    };

}
#endif
#endif 