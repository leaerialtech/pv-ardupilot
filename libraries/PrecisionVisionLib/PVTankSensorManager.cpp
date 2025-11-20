#include "PVTankSensorManager.h"

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

namespace PrecisionVision
{

    PVTankSensorManager::PVTankSensorManager()
    {
        if (_singleton) {
        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                AP_HAL::panic("Too many tanksensormanagers");
        #endif
        return;
    }
    _singleton = this;


    };
    
    PVTankSensorManager *PVTankSensorManager::_singleton;
    PVTankSensorManager *PVTankSensorManager::get_singleton()
    {   
        return _singleton;
    }

    TankSensorState PVTankSensorManager::get_tank_sensor_status(){
        return this->_tank_sensor_status;
    }

    void PVTankSensorManager::set_delegate(PVTankSensorDelegate* delegate)
    {
        this->_delegate = delegate;
    }
    
    //PrecisionVision: TankSensor is currently wired to an aux pin on the carrier board
    //it sends high signal when the tank is below the level (warning) 
    void PVTankSensorManager::pv_update_tank_sensor(int tankpin)
    {
        static bool inBootDelay = true;
        if(tankpin <= 0){
            return;
        }
   
        AP_HAL::get_HAL().gpio->pinMode(((uint8_t)tankpin), HAL_GPIO_INPUT);
        uint8_t pin_state = AP_HAL::get_HAL().gpio->read((uint8_t)tankpin);   //high signal means tank is full, 0 signal means tank is low. 


        //uncomment for console debug
        //if(cnt++ > 100)
        //{
        //    cnt = 0;
        //    gcs().send_text(MAV_SEVERITY_WARNING, "pin %d is %s", (uint8_t)g.pvtank_pin, (pin_state > 0 ?  "HIGH" : "LOW"));
        //}


        //if we are transitioning to the low state, we alert the user, and if we happen to be in a guided/auto flight mode,
        //we kick out into brake mode. 
        ////right now, the tank sensor is always on, may need to set this up with a parameter so that we can
        //change the configuration, particularly if we get a new board. 
          
        //if we WERE full, and now we are not, send it into brake mode! 
        if(_tank_sensor_status == TankSensorState::TANK_FULL  && pin_state == 0){
                //BUT only do so if A.) we are flying, B.) in regular auto mode (not loiter, takeoff, landing, or rtl, etc.)
                //and C.) at least 8 seconds has passed since bootup, just to make sure everything powers up right

                if(inBootDelay && AP_HAL::millis() > 20000){ 
                    inBootDelay = false; 
                }
                
                if(!inBootDelay)
                {
                    if(this->_delegate){
                        this->_delegate->handleTankSensorTriggered();
                    }
                }
            }
        _tank_sensor_status = pin_state > 0 ? TankSensorState::TANK_FULL : TankSensorState::TANK_EMPTY; 
    }
}