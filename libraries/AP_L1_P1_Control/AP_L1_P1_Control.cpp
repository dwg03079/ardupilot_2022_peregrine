#include <AP_HAL/AP_HAL.h>
#include "AP_L1_P1_Control.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_L1_P1_Control::var_info[] = {
    // @Param: 2SRV_TCONST
    // @DisplayName: Time Constant
    // @Description: 
    // @Range: 0.4 1.0
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("POS_TCONST",      0, AP_L1_P1_Control, gains.tau,       0.5f),

    // index 1 to 3 reserved for old PID values

    // @Param: 2SRV_RMAX
    // @DisplayName: Maximum Rate
    // @Description: 
    // @Range: 0 180
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POS_RMAX",   4, AP_L1_P1_Control, gains.rmax_pos,       0),

    // index 5, 6 reserved for old IMAX, FF

    // @Param: _RATE_P
    // @DisplayName: rate controller P gain
    // @Description: 
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: rate controller I gain
    // @Description: 
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: rate controller I gain maximum
    // @Description: 
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: rate controller D gain
    // @Description: 
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: rate controller feed forward
    // @Description: 
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: rate controller target frequency in Hz
    // @Description: 
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: rate controller error frequency in Hz
    // @Description: 
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: rate controller derivative frequency in Hz
    // @Description: 
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: slew rate limit
    // @Description: 
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_L1_P1_Control, AC_PID),

    AP_GROUPEND
};

void AP_L1_P1_Control::update_pid(float error)
{
    P1_pid.update_error(error, true);
}




void AP_L1_P1_Control::reset_I()
{
    _pid_info.I = 0;
    P1_pid.reset_I();
}

void AP_L1_P1_Control::decay_I()
{
    // this reduces integrator by 95% over 2s
    P1_pid_info.I *= 0.995f;
    P1_pid.set_integrator(P1_pid.get_i() * 0.995);
}