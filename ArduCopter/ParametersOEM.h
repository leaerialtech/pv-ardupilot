#pragma once
#include <AP_Param/AP_Param.h>

////////////////////////
//PrecisionVision
//Additional PrecisionVision custom parameters 
///////////////////////
class ParametersOEM {
public:
    // Brake flightmode should run loiter control (but ignore sticks)
    AP_Int8  brk_use_loiter;

    static const AP_Param::GroupInfo var_info[];
};
