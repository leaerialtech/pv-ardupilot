#include "PVResumePointCreator.h"

#include <AP_Mission/AP_Mission.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_AHRS/AP_AHRS.h>

#include "../ArduCopter/Copter.h"


namespace PrecisionVision
{

    PVResumePointCreator::PVResumePointCreator(AP_Mission &mission, const AP_AHRS& ahrs, const AC_Sprayer &sprayer) : 
      mission_(mission),
      ahrs_(ahrs), 
      sprayer_(sprayer)
    {

    }

    //needs an instance of copter to extract tthe wpt from
    ERRCODE PVResumePointCreator::CreateResumePointAtCurrentUavLocationAndState()
    {

        // // set new waypoint to current location
        // Location temp_loc;
     //   const AP_AHRS &ahrs = AP::ahrs();
        // if(!ahrs.get_position(temp_loc)){
        //     return MAV_RESULT_FAILED;
        // }
        int sprayState = this->sprayer_.spraying();
        
        //float altitudeEstimate = 0;
        //ahrs.get_hagl(altitudeEstimate);
        //float inCm = altitudeEstimate * 100.0;
        //temp_loc.set_alt_cm(inCm, Location::AltFrame::ABOVE_TERRAIN);

        int idx = mission_.get_prev_nav_cmd_with_wp_index();
        if (idx <= 0 || idx > mission_.num_commands())
        {
            return ERRCODE::PV_ERC_FAIL;
        }

        // create new mission command
        AP_Mission::Mission_Command wp_cmd = {};
        AP_Mission::Mission_Command spry_cmd = {};
        AP_Mission::Mission_Command spd_cmd = {};
        AP_Mission::Mission_Command prevWptCmd = {};

        if (!mission_.read_cmd_from_storage(idx, prevWptCmd))
        {
            return ERRCODE::PV_ERC_FAIL;
        }

        if (prevWptCmd.id == MAV_CMD_NAV_TAKEOFF)
        {
      //      return ERRCODE::PV_ERC_FAIL; //we do nothing, I originally wanted to use "Failed" but
            //it produces a message on QGC and for precisionvision purposes, we don't need to overly complicate.
        }

        Location temp_loc;
        if (!ahrs_.get_position(temp_loc))
        {
            return ERRCODE::PV_ERC_FAIL;
        }

    

        Location::AltFrame lastWptAltframe_ArduValue = prevWptCmd.content.location.get_alt_frame();
       // printf("Last ardu-alt frame %d", (int)lastWptAltframe_ArduValue); //unfortunately, the altframe here is NOT the same as the mavlink-alt enum used by PV/QGC. 
        int32_t altCm = 0;
        if (!prevWptCmd.content.location.get_alt_cm(lastWptAltframe_ArduValue, altCm)){
            return ERRCODE::PV_ERC_FAIL;
        }

        //WARNING: do not confuse nav_delay with "content.delay" which I guess is conditional.
        //caution, you can only choose one "type" - so if you set cmd.content.location, you can't use any of the other ones
        //also - its seems "p1" in ardupilot specifies the hold/delay time that would be put in the actual mavlink command.
        //using content.location.delay or content.location.nav_delay must be other message types. 

        wp_cmd.id = MAV_CMD_NAV_WAYPOINT;

        temp_loc.set_alt_cm(altCm, lastWptAltframe_ArduValue);
        wp_cmd.content.location = temp_loc;
        // make the new command to a waypoint

        //not sure what actually is going to make this trigger, filling it all out.
        spry_cmd.id = MAV_CMD_USER_1;

        spry_cmd.p1 = sprayState;
        AP_Mission::User1_Command sprayInfo;
        sprayInfo.param1 = sprayState > 0 ? 1 : 0; //ignore heading checks if sprayon! 
        sprayInfo.param2 = sprayState;

        spry_cmd.content.user1 = sprayInfo;

        AP_Mission::Change_Speed_Command speedchange;
        spd_cmd.id = MAV_CMD_DO_CHANGE_SPEED;
        speedchange.speed_type = 1; //specify we want groundspeed
        speedchange.target_ms = -1; //indicate no change for now  // packet.param4; //the packet from user should be meters per second
        speedchange.throttle_pct = -1;
        spd_cmd.content.speed = speedchange;

        //int current_wp_idx = copter.mode_auto.mission.get_current_nav_index();
        int current_wp_idx = mission_.get_current_nav_index();
        AP_Mission::Mission_Command arr[3] = {wp_cmd, spry_cmd, spd_cmd}; // spd_cmd, spry_cmd };

        // save command
        if (this->insert_cmds_(current_wp_idx, arr, 3))
        {
            hal.console->printf("inserted waypoint");
            return ERRCODE::PV_ERC_SUCCESS;
        }
         return ERRCODE::PV_ERC_FAIL;
    }
    //PrecisionVison code, used to insert resume waypoints
    bool PVResumePointCreator::insert_cmds_(uint16_t index, AP_Mission::Mission_Command cmd[], int numCmds)
    {
        if (cmd == 0 || numCmds <= 0)
        {
            return false;
        }
        int i = 0;

        // sanity check index
        if (index >= (unsigned)mission_.num_commands())
        {
            for (i = 0; i < numCmds; i++)
            {
                mission_.add_cmd(cmd[i]);
            }
        }

        AP_Mission::Mission_Command temp_cmd;
        //copy over our last n commands to make room for the ones we will be inserting

        int baseIdx = mission_.num_commands() - numCmds;
        i = 0;
        for (int j = 0; j < numCmds; j++)
        {
            if (!mission_.read_cmd_from_storage(baseIdx + j, temp_cmd))
            {
                return false;
            }
            if (!mission_.add_cmd(temp_cmd))
            {
                return false;
            }
        }

        //push out all cmds above the insert point
        //command total has now increased, so we -N so that we are looing at the second to last command
        //of the original set
        i = 0;
        for (i = mission_.num_commands() - (2 + numCmds); i >= index; i--)
        {
            if (!mission_.read_cmd_from_storage(i, temp_cmd))
            {
                return false;
            }
            if (!mission_.replace_cmd(i + numCmds, temp_cmd))
            {
                return false;
            }
        }

        //finally, replace the new command
        i = 0;
        for (i = 0; i < numCmds; i++)
        {
            if (!mission_.replace_cmd(index + i, cmd[i]))
            {
                return false;
            }
        }

        /*
    Mission_Command* checkCmds = new Mission_Command[_cmd_total];
    for(int z=0; z < _cmd_total; z++){
        read_cmd_from_storage(z, checkCmds[z]);
    }

    for(int q=0; q < _cmd_total; q++){
        if(checkCmds[q].index != q){
            int zz = 21;
            zz++; 
        }
    }
*/

        return true;
    }

}