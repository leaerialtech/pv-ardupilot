#ifndef PV_POSITION_H
#define PV_POSITION_H

#include <memory>

namespace PrecisionVision
{
    //ValueObject to hold navigational properties of a specific lat/lng  global coords 
    class PVPosition
    {
        protected:
            float latitude_;
            float longitude_;
            float altitude_meters_;
            int   altitude_reference_frame_;
            
            PVPosition(float latitude, float longitude, float altMeters, int ref_frame);

            public: 
            static std::unique_ptr<PVPosition> Create(float latitude, float longitude, float altMeters, int ref_frame);

            float getLatitude();
            float getLongitude();
            float getAltitudeMeters();
            int getAltReferenceFrame();
            
            bool isValid();
    };
}
#endif