#ifndef PVTANK_SENSOR_MANAGER_xx_H
#define PVTANK_SENSOR_MANAGER_xx_H


#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

namespace PrecisionVision
{

      typedef enum class _TankSensorState {
            SENSOR_UNAVAILABLE = -1,
            TANK_EMPTY = 0,
            TANK_FULL = 1,
        } TankSensorState;


    //abstract class for delegate pattern 
    class PVTankSensorDelegate
    {  
        public:
        virtual void handleTankSensorTriggered()=0;
    };


    class PVTankSensorManager  
    {
        /* Do not allow copies */

            public: 
            PVTankSensorManager();  

            CLASS_NO_COPY(PVTankSensorManager);

           
            static PVTankSensorManager *get_singleton();
            static PVTankSensorManager *_singleton;


            void pv_update_tank_sensor(int tankPin); //call this in the main loop to send status to gcs 
            TankSensorState get_tank_sensor_status();
            void set_delegate(PVTankSensorDelegate* delegate);

          
        private:
           PVTankSensorDelegate* _delegate;
           TankSensorState _tank_sensor_status;
 
    };
}
#endif 