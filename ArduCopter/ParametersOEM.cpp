#warning "Compiling ParametersOEM.cpp"
#include "ParametersOEM.h"

const AP_Param::GroupInfo ParametersOEM::var_info[] = {
    // @Param: BRK_USE_LOIT    -- make brake flight mode actually run loiter controller, but ignore sticks 
    //this is useful because stock brake mode does not use loiter_brake params, and it seeds altitude differently
    AP_GROUPINFO("BRK_USE_LOIT",    3, ParametersOEM, brk_use_loiter, 0),
    AP_GROUPEND
};
