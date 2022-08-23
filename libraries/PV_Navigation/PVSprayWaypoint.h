#ifndef PVWAYPOINT_H
#define PVWAYPOINT_H

#include <memory>
#include "PVPosition.h"

namespace PrecisionVision
{
    //valueobject representing a spray point in precisionvision
    class PVSprayWaypoint
    {
        public:
        PVPosition _navPoint;
        int sprayStateInTransit_; //what sprayer state should we be in when departing the waypoint (until the next waypoint or override command reached) 
        int spotSprayState_; 
        int navDelaySec_;   //how long do we sit at the waypoint (not spraying) - if/when coupled with spot spray it could be considerd "pre-spray delay" 
        int spotSprayDelaySec_; //how long do we turn the spray on over this waypoint without moving stationary 
        
        PVSprayWaypoint(PVPosition nav, int navDelaySec, int departingSprayState, int sprayDelaySec);
        
        PVPosition getPosition();
        int getNavigationDelaySeconds() const;
        int getSpotSprayDelaySeconds() const;
        int getSprayStateUponDeparting() const ;
    };
}
#endif