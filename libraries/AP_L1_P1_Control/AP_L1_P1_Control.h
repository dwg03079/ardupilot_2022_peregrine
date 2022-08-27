#pragma once

#include <AC_PID/AC_PID.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

class AP_L1_P1_Control {
public:
    AP_L1_P1_Control()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_L1_P1_Control(const AP_L1_P1_Control& other) = delete;
    AP_L1_P1_Control& operator=(const AP_L1_P1_Control&) = delete;
    
    //custom
    
    float dist_error;
    void update_pid(float error);




    //copied

    void reset_I();
    
    void decay_I();

    const AP_PIDInfo& get_pid_info(void) const
    {
        return P1_pid_info;
    }

    static const struct AP_Param::GroupInfo var_info[];

    // tuning accessors
    void kP(float v) { rate_pid.kP().set(v); }
    void kI(float v) { rate_pid.kI().set(v); }
    void kD(float v) { rate_pid.kD().set(v); }
    void kFF(float v) { rate_pid.ff().set(v); }

    AP_Float& kP(void) { return rate_pid.kP(); }
    AP_Float& kI(void) { return rate_pid.kI(); }
    AP_Float& kD(void) { return rate_pid.kD(); }
    AP_Float& kFF(void) { return rate_pid.ff(); }

private:
    
    //custom, P1_Control
    AC_PID P1_pid{0.08, 0.15, 0, 0.345, 0.666, 3, 0, 12, 0.02, 150, 1};

    AP_PIDInfo P1_pid_info;
};
