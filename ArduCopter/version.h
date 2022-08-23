/////////////////////////////////////////////////////////////////
//Modified by Leading Edge Aerial Technologies, LLC. (Apr 2021)//
/////////////////////////////////////////////////////////////////

#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif


//Version Number  PrecisionVision Version Num
//#include "ap_version.h"

#define THISFIRMWARE "PrecisionVision V1.2.0"   
//i think this can only be 8 chars

//7 charh hash string
#define PVHASHSTR "1.2.0" //FOR THIS VERSION, BE SURE NOT TO INCLUDE THE "PV" -- this tells early versions of GCS to not wait for resume

// the following line is parsed by the autotest scripts
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

#define FIRMWARE_VERSION 1,2,0, FW_TYPE
#define FW_MAJOR 1
#define FW_MINOR 2
#define FW_PATCH 0

#include <AP_Common/AP_FWVersionDefine.h>