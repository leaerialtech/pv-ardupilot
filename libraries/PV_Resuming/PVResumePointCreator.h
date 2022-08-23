#ifndef PVRESUME_POINT_CREATOR_xx_H
#define PVRESUME_POINT_CREATOR_xx_H

#include <AP_Mission/AP_Mission.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Sprayer/AC_Sprayer.h>


namespace PrecisionVision
{
    enum ERRCODE
    {
        PV_ERC_FAIL =0,
        PV_ERC_SUCCESS = 1,
    };

    class PVResumePointCreator
    {
        
        protected:
        

        private: 
        AP_Mission& mission_;
        const AC_Sprayer& sprayer_;
        const AP_AHRS& ahrs_;

        //custom precisionvision code for inserting resume waypoint.
        bool insert_cmds_(uint16_t index,  AP_Mission::Mission_Command cmd[], int numCmds=1);

        public: 
            //constructors
            PVResumePointCreator(AP_Mission& mission, const AP_AHRS& ahrs, const AC_Sprayer &sprayer);

            //
            ERRCODE CreateResumePointAtCurrentUavLocationAndState();

    };

    
}
#endif 