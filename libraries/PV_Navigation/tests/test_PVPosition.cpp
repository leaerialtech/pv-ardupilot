#include <AP_gtest.h>
#include <memory>

#include "../PVPosition.h"

#include <AP_ADSB/AP_ADSB.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace PrecisionVision;

//Write a separate test for every IF, And, Or, Case, For and While condition within a method.
TEST(PV_Navigation, LatShouldBeWithinRealworldBounds)
{
    //arrange//
    std::unique_ptr<PVPosition> result1;
    std::unique_ptr<PVPosition> result2;
    std::unique_ptr<PVPosition> result3;    
    std::unique_ptr<PVPosition> result4;
    std::unique_ptr<PVPosition> result5;
    std::unique_ptr<PVPosition> result6;
    std::unique_ptr<PVPosition> result7;    
    std::unique_ptr<PVPosition> result8;

    //act//
    result1 = PVPosition::Create(-90.01, 0, 0, 0);
    result2 = PVPosition::Create(-89.999, 0, 0, 0);
    result3 = PVPosition::Create(-90.0, 0, 0, 0);
    result4 = PVPosition::Create(-99,1,2,3);

    result5 = PVPosition::Create(90.01, 0, 0, 0);
    result6 = PVPosition::Create(89.999, 0, 0, 0);
    result7 = PVPosition::Create(90.0, 0, 0, 0);
    result8 = PVPosition::Create(99,1,2,3);

    //assert//    
    ASSERT_FLOAT_EQ(result1->getLatitude(),-90.00);
    ASSERT_FLOAT_EQ(result2->getLatitude(),-89.999);
    ASSERT_FLOAT_EQ(result3->getLatitude(),-90.0);
    ASSERT_FLOAT_EQ(result4->getLatitude(),-90.0);

    ASSERT_FLOAT_EQ(result5->getLatitude(), 90.00);
    ASSERT_FLOAT_EQ(result6->getLatitude(), 89.999);
    ASSERT_FLOAT_EQ(result7->getLatitude(), 90.0);
    ASSERT_FLOAT_EQ(result8->getLatitude(), 90.0);
}

TEST(PVPosition, LngShouldBeWithinRealworldBounds)
{
    //arrange//
    std::unique_ptr<PVPosition> result1;
    std::unique_ptr<PVPosition> result2;
    std::unique_ptr<PVPosition> result3;    
    std::unique_ptr<PVPosition> result4;
    std::unique_ptr<PVPosition> result5;
    std::unique_ptr<PVPosition> result6;
    std::unique_ptr<PVPosition> result7;    
    std::unique_ptr<PVPosition> result8;

    //act//
    result1 = PVPosition::Create(0,-180.01, 0, 0);
    result2 = PVPosition::Create(0,-179.999, 0, 0);
    result3 = PVPosition::Create(0,-180.0, 0, 0);
    result4 = PVPosition::Create(0,-189,1,2);

    result5 = PVPosition::Create(0,180.01, 0, 0);
    result6 = PVPosition::Create(0,179.999, 0, 0);
    result7 = PVPosition::Create(0,180.0, 0, 0);
    result8 = PVPosition::Create(0,189,1,2);

    //assert//    
    ASSERT_FLOAT_EQ(result1->getLongitude(),-180.00);
    ASSERT_FLOAT_EQ(result2->getLongitude(),-179.999);
    ASSERT_FLOAT_EQ(result3->getLongitude(),-180.0);
    ASSERT_FLOAT_EQ(result4->getLongitude(),-180.0);
    ASSERT_FLOAT_EQ(result5->getLongitude(), 180.00);
    ASSERT_FLOAT_EQ(result6->getLongitude(), 179.999);
    ASSERT_FLOAT_EQ(result7->getLongitude(), 180.0);
    ASSERT_FLOAT_EQ(result8->getLongitude(), 180.0);
}

AP_GTEST_MAIN()