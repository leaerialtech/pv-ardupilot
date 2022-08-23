#include  "PVPosition.h"
#include <exception>    

namespace PrecisionVision
{
    
    // namespace
    PVPosition::PVPosition(float latitude, float longitude, float altMeters, int ref_frame)
    {
        this->latitude_ = latitude;
        this->longitude_ = longitude;

        //lat/lng should always be within -90,+90 and -180,+180 respectively 
        if(latitude < -90.0){
            this->latitude_ = -90.0;
        }
        if(latitude > 90.0){
            this->latitude_ = 90.0;
        }
        if(longitude > 180.0){
            this->longitude_ = 180.0;
        }
        if(longitude < -180.0){
            this->longitude_ = -180;
        }
    }


    std::unique_ptr<PVPosition> PVPosition::Create(float latitude, float longitude, float altMeters, int ref_frame)
    {
        PVPosition* ptr =  new (std::nothrow) PVPosition(latitude, longitude, altMeters, ref_frame);
        if(ptr == nullptr){return nullptr;}
        return std::unique_ptr<PVPosition>(ptr);     
    }

    float PVPosition::getLatitude(){
        return this->latitude_;
    }

    float PVPosition::getLongitude(){
        return this->longitude_;
    }
    
}