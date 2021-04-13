/////////////////////////////////////////////////////////////////
//Modified by Leading Edge Aerial Technologies, LLC. (Apr 2021)//
/////////////////////////////////////////////////////////////////

#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"
#define FW_NAME "PrecisionVision"
#define FW_MAJOR 1
#define FW_MINOR 1
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL


#define STRINGIFY(x) STRINGIFY_(x)
#define STRINGIFY_(x) #x
#define THISFIRMWARE   STRINGIFY(SW_VERSION_MAJOR) "." STRINGIFY(SW_VERSION_MINOR) "." STRINGIFY(SW_VERSION_MAINTENANCE)
