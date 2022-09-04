#include "Plane.h"

#include "Attack.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo Attack::var_info[] = {

    // @Param: P_P
    AP_GROUPINFO("P_P", 0, Attack, pitch_rate_pid_kp, 0.0f),
    // @Param: P_I
    AP_GROUPINFO("P_I", 1, Attack, pitch_rate_pid_ki, 0.0f),
    // @Param: P_D
    AP_GROUPINFO("P_D", 2, Attack, pitch_rate_pid_kd, 0.0f),
    // @Param: P_FF
    AP_GROUPINFO("P_FF", 3, Attack, pitch_rate_pid_kff, 0.0f),
    // @Param: P_IMAX
    AP_GROUPINFO("P_IMAX", 4, Attack, pitch_rate_pid_kimax, 0.0f),
    // @Param: P_FLTT
    AP_GROUPINFO("P_FLTT", 5, Attack, pitch_rate_pid_filt_T_hz, AC_PID_TFILT_HZ_DEFAULT),
    // @Param: P_FLTE
    AP_GROUPINFO("P_FLTE", 6, Attack, pitch_rate_pid_filt_E_hz, AC_PID_EFILT_HZ_DEFAULT),
    // @Param: P_FLTD
    AP_GROUPINFO("P_FLTD", 7, Attack, pitch_rate_pid_filt_D_hz, AC_PID_DFILT_HZ_DEFAULT),
    // @Param: P_SMAX
    AP_GROUPINFO("P_SMAX", 8, Attack, pitch_rate_pid_slew_rate_max, 0.0f),



    // @Param: R_P
    AP_GROUPINFO("R_P", 9, Attack, roll_rate_pid_kp, 0.0f),
    // @Param: R_I
    AP_GROUPINFO("R_I", 10, Attack, roll_rate_pid_ki, 0.0f),
    // @Param: R_D
    AP_GROUPINFO("R_D", 11, Attack, roll_rate_pid_kd, 0.0f),
    // @Param: R_FF
    AP_GROUPINFO("R_FF", 12, Attack, roll_rate_pid_kff, 0.0f),
    // @Param: R_IMAX
    AP_GROUPINFO("R_IMAX", 13, Attack, roll_rate_pid_kimax, 0.0f),
    // @Param: R_FLTT
    AP_GROUPINFO("R_FLTT", 14, Attack, roll_rate_pid_filt_T_hz, AC_PID_TFILT_HZ_DEFAULT),
    // @Param: R_FLTE
    AP_GROUPINFO("R_FLTE", 15, Attack, roll_rate_pid_filt_E_hz, AC_PID_EFILT_HZ_DEFAULT),
    // @Param: R_FLTD
    AP_GROUPINFO("R_FLTD", 16, Attack, roll_rate_pid_filt_D_hz, AC_PID_DFILT_HZ_DEFAULT),
    // @Param: R_SMAX
    AP_GROUPINFO("R_SMAX", 17, Attack, roll_rate_pid_slew_rate_max, 0.0f),



    // @Param: Y_P
    AP_GROUPINFO("Y_P", 18, Attack, yaw_rate_pid_kp, 0.0f),
    // @Param: Y_I
    AP_GROUPINFO("Y_I", 19, Attack, yaw_rate_pid_ki, 0.0f),
    // @Param: Y_D
    AP_GROUPINFO("Y_D", 20, Attack, yaw_rate_pid_kd, 0.0f),
    // @Param: Y_FF
    AP_GROUPINFO("Y_FF", 21, Attack, yaw_rate_pid_kff, 0.0f),
    // @Param: Y_IMAX
    AP_GROUPINFO("Y_IMAX", 22, Attack, yaw_rate_pid_kimax, 0.0f),
    // @Param: Y_FLTT
    AP_GROUPINFO("Y_FLTT", 23, Attack, yaw_rate_pid_filt_T_hz, AC_PID_TFILT_HZ_DEFAULT),
    // @Param: Y_FLTE
    AP_GROUPINFO("Y_FLTE", 24, Attack, yaw_rate_pid_filt_E_hz, AC_PID_EFILT_HZ_DEFAULT),
    // @Param: Y_FLTD
    AP_GROUPINFO("Y_FLTD", 25, Attack, yaw_rate_pid_filt_D_hz, AC_PID_DFILT_HZ_DEFAULT),
    // @Param: Y_SMAX
    AP_GROUPINFO("Y_SMAX", 26, Attack, yaw_rate_pid_slew_rate_max, 0.0f),



    // @Param: RC_ch
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("RC_ch",    27, Attack, rc_channel, 7U),

    // @Param: S1_ch
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Rang
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("S1_ch",    28, Attack, servo_01_channel, 9U),

    // @Param: S1_lock
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("S1_lock",    29, Attack, servo_01_pwm_lock, 2000U),

    // @Param: S1_unlock
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("S1_unlock",    30, Attack, servo_01_pwm_unlock, 1000U),

    // @Param: S2_ch
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("S2_ch", 31, Attack, servo_02_channel, 10U),

    // @Param: S2_lock
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("S2_lock", 32, Attack, servo_02_pwm_lock, 1000U),
    
    // @Param: S2_unlock
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("S2_unlock", 33, Attack, servo_02_pwm_unlock, 2000U),


    // @Param: CLIMB_R
    // @DisplayName: 
    // @Description: FBWB_CLIMB_RATE
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("CLIMB_R", 34, Attack, flybywire_climb_rate, 50.0f),

    // @Param: LIM_PMAX
    // @DisplayName: 
    // @Description: LIM_PITCH_MAX
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("LIM_PMAX", 35, Attack, pitch_limit_max_cd, 6000),
    
    // @Param: LIM_PMIN
    // @DisplayName: 
    // @Description: LIM_PITCH_MIN
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("LIM_PMIN", 36, Attack, pitch_limit_min_cd, -8900),

    // @Param: LIM_R_CD
    // @DisplayName: 
    // @Description: LIM_ROLL_CD
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("LIM_R_CD", 37, Attack, roll_limit_cd, 8900),

    // @Param: FBW_MIN
    // @DisplayName: 
    // @Description: ARSPD_FBW_MIN
    // @Units: 
    // @Range: 1 32767
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("FBW_MIN", 38, Attack, airspeed_min, 9),

    // @Param: FBW_MAX
    // @DisplayName: 
    // @Description: ARSPD_FBW_MAX
    // @Units: 
    // @Range: 1 32767
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("FBW_MAX", 39, Attack, airspeed_max, 32767),


    // @Param: CLMB_MAX
    // @DisplayName: 
    // @Description: CLIMB_MAX
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("CLMB_MAX", 40, Attack, TECS_maxClimbRate, 50.0f),
    
    // @Param: T_CONST
    // @DisplayName: 
    // @Description: TIME_CONST
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("T_CONST", 41, Attack, TECS_timeConst, 5.0f),
    
    // @Param: SINK_MAX
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("SINK_MAX", 42, Attack, TECS_maxSinkRate, 340.0f),
    
    // @Param: PIT_MAX
    // @DisplayName: 
    // @Description: PITCH_MAX
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("PIT_MAX", 43, Attack, TECS_pitch_max, 60),
    
    // @Param: PIT_MIN
    // @DisplayName: 
    // @Description: PITCH_MIN
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("PIT_MIN", 44, Attack, TECS_pitch_min, 89),

    // @Param: L1_PRD
    // @DisplayName: 
    // @Description: PITCH_MIN
    // @Units: 
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("L1_PRD", 45, Attack, _L1_period, 17),
    
    AP_GROUPEND
};

void Attack::attack_update()
{
    if (check_backup == false) {
        backup();
        check_backup = true;
    }

    RC_Channel* selchan = rc().channel(rc_channel - 1);
    if (selchan == nullptr) {
        return;
    }

    constrain_float(servo_01_pwm_lock, 800UL, 2200UL);
    constrain_float(servo_01_pwm_unlock, 800UL, 2200UL);
    constrain_float(servo_02_pwm_lock, 800UL, 2200UL);
    constrain_float(servo_02_pwm_unlock, 800UL, 2200UL);

    if (selchan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH) {
        //selchan->get_radio_in() >= RC_Channel::AUX_PWM_TRIGGER_HIGH
        if (state_extra_pid == false) {
            gcs().send_text(MAV_SEVERITY_INFO, "Do separate Wings");

            change_extra_pid();
            if (servo_01_channel >= 1 && servo_01_channel <= 16) {
                if (set_servo(servo_01_channel, servo_01_pwm_unlock)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Servo 01 Unlocked");
                }
                else {
                    gcs().send_text(MAV_SEVERITY_INFO, "Failed Servo 01 Unlock");
                }
            }
            if (servo_02_channel >= 1 && servo_02_channel <= 16) {
                if (set_servo(servo_02_channel, servo_02_pwm_unlock)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Servo 02 Unlocked");
                }
                else {
                    gcs().send_text(MAV_SEVERITY_INFO, "Failed Servo 02 Unlock");
                }
            }
            state_extra_pid = true;
        }
    } else if (selchan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::LOW) {
        if (state_extra_pid == true) {
            change_base_pid();
            if (servo_01_channel >= 1 && servo_01_channel <= 16) {
                if (set_servo(servo_01_channel, servo_01_pwm_lock)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Servo 01 Locked");
                }
                else {
                    gcs().send_text(MAV_SEVERITY_INFO, "Failed Servo 01 Lock");
                }
            }
            if (servo_02_channel >= 1 && servo_02_channel <= 16) {
                if (set_servo(servo_02_channel, servo_02_pwm_lock)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Servo 02 Locked");
                }
                else {
                    gcs().send_text(MAV_SEVERITY_INFO, "Failed Servo 02 Lock");
                }
            }
            state_extra_pid = false;
            state_wp_ignore = false;
        }
    } else {
        
    }

    if (state_wp_ignore == true) {
        uint8_t _channel = 3;
        SRV_Channel* c = SRV_Channels::srv_channel(_channel - 1);
        if (c == nullptr) {
        }
        switch (c->get_function())
        {
        case SRV_Channel::k_none:
        case SRV_Channel::k_manual:
        case SRV_Channel::k_sprayer_pump:
        case SRV_Channel::k_sprayer_spinner:
        case SRV_Channel::k_gripper:
        case SRV_Channel::k_rcin1 ... SRV_Channel::k_rcin16: // rc pass-thru
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "ServoRelayEvent: Channel %d is already in use", _channel);
        }

        uint16_t pwm = 950;
        c->set_output_pwm(pwm);
        c->ignore_small_rcin_changes();
    }

    if (plane.get_mode() == Mode::Number::AUTO) {
        check_command(plane.mission.get_current_do_cmd());
    } else {
        lock_limit();
        state_limit = true;
        state_wp_ignore = false;
        plane.TECS_controller.attack_hgt_dem = false;
        plane.nav_controller->L1_state_wp_recover();
        plane.aparm.throttle_max = 100;
    }
}

//copy & fix - AP_ServoRelayEvents
bool Attack::set_servo(uint8_t _channel, uint16_t pwm)
{
    SRV_Channel* c = SRV_Channels::srv_channel(_channel - 1);
    if (c == nullptr) {
        return false;
    }
    switch (c->get_function())
    {
    case SRV_Channel::k_none:
    case SRV_Channel::k_manual:
    case SRV_Channel::k_sprayer_pump:
    case SRV_Channel::k_sprayer_spinner:
    case SRV_Channel::k_gripper:
    case SRV_Channel::k_rcin1 ... SRV_Channel::k_rcin16: // rc pass-thru
        break;
    default:
        gcs().send_text(MAV_SEVERITY_INFO, "ServoRelayEvent: Channel %d is already in use", _channel);
        return false;
    }
    
    c->set_output_pwm(pwm);
    c->ignore_small_rcin_changes();
    return true;
}

void Attack::check_command(const AP_Mission::Mission_Command& cmd)
{
    

    if (cmd.id == MAV_CMD_USER_4 && state_limit == true) {
        unlock_limit();
        state_limit = false;
        state_wp_ignore = true;
        plane.nav_controller->L1_state_wp_ignore();
        plane.TECS_controller.attack_hgt_dem = true;
        //plane.TECS_controller.next_wp = plane.next_WP_loc;
        plane.aparm.throttle_max = 0;
    } else if (cmd.id == MAV_CMD_USER_5) {
        lock_limit();
        state_limit = true;
        state_wp_ignore = false;
        plane.TECS_controller.attack_hgt_dem = false;
    }
}

void Attack::change_extra_pid()
{
    //backup first pid
    base_pitch_rate_pid._kp = plane.pitchController.rate_pid.kP();
    base_pitch_rate_pid._ki = plane.pitchController.rate_pid.kI();
    base_pitch_rate_pid._kd = plane.pitchController.rate_pid.kD();
    base_pitch_rate_pid._kff = plane.pitchController.rate_pid.ff();
    base_pitch_rate_pid._kimax = plane.pitchController.rate_pid.kIMAX();
    base_pitch_rate_pid._filt_T_hz = plane.pitchController.rate_pid.filt_T_hz();
    base_pitch_rate_pid._filt_E_hz = plane.pitchController.rate_pid.filt_E_hz();
    base_pitch_rate_pid._filt_D_hz = plane.pitchController.rate_pid.filt_D_hz();
    base_pitch_rate_pid._slew_rate_max = plane.pitchController.rate_pid.slew_limit();

    base_roll_rate_pid._kp = plane.rollController.rate_pid.kP();
    base_roll_rate_pid._ki = plane.rollController.rate_pid.kI();
    base_roll_rate_pid._kd = plane.rollController.rate_pid.kD();
    base_roll_rate_pid._kff = plane.rollController.rate_pid.ff();
    base_roll_rate_pid._kimax = plane.rollController.rate_pid.kIMAX();
    base_roll_rate_pid._filt_T_hz = plane.rollController.rate_pid.filt_T_hz();
    base_roll_rate_pid._filt_E_hz = plane.rollController.rate_pid.filt_E_hz();
    base_roll_rate_pid._filt_D_hz = plane.rollController.rate_pid.filt_D_hz();
    base_roll_rate_pid._slew_rate_max = plane.rollController.rate_pid.slew_limit();

    base_yaw_rate_pid._kp = plane.yawController.rate_pid.kP();
    base_yaw_rate_pid._ki = plane.yawController.rate_pid.kI();
    base_yaw_rate_pid._kd = plane.yawController.rate_pid.kD();
    base_yaw_rate_pid._kff = plane.yawController.rate_pid.ff();
    base_yaw_rate_pid._kimax = plane.yawController.rate_pid.kIMAX();
    base_yaw_rate_pid._filt_T_hz = plane.yawController.rate_pid.filt_T_hz();
    base_yaw_rate_pid._filt_E_hz = plane.yawController.rate_pid.filt_E_hz();
    base_yaw_rate_pid._filt_D_hz = plane.yawController.rate_pid.filt_D_hz();
    base_yaw_rate_pid._slew_rate_max = plane.yawController.rate_pid.slew_limit();

    //set second pid
    plane.pitchController.rate_pid.kP(pitch_rate_pid_kp);
    plane.pitchController.rate_pid.kI(pitch_rate_pid_ki);
    plane.pitchController.rate_pid.kD(pitch_rate_pid_kd);
    plane.pitchController.rate_pid.ff(pitch_rate_pid_kff);
    plane.pitchController.rate_pid.imax(pitch_rate_pid_kimax);
    plane.pitchController.rate_pid.filt_T_hz(pitch_rate_pid_filt_T_hz);
    plane.pitchController.rate_pid.filt_E_hz(pitch_rate_pid_filt_E_hz);
    plane.pitchController.rate_pid.filt_D_hz(pitch_rate_pid_filt_D_hz);
    plane.pitchController.rate_pid.slew_limit(pitch_rate_pid_slew_rate_max);

    plane.rollController.rate_pid.kP(roll_rate_pid_kp);
    plane.rollController.rate_pid.kI(roll_rate_pid_ki);
    plane.rollController.rate_pid.kD(roll_rate_pid_kd);
    plane.rollController.rate_pid.ff(roll_rate_pid_kff);
    plane.rollController.rate_pid.imax(roll_rate_pid_kimax);
    plane.rollController.rate_pid.filt_T_hz(roll_rate_pid_filt_T_hz);
    plane.rollController.rate_pid.filt_E_hz(roll_rate_pid_filt_E_hz);
    plane.rollController.rate_pid.filt_D_hz(roll_rate_pid_filt_D_hz);
    plane.rollController.rate_pid.slew_limit(roll_rate_pid_slew_rate_max);

    plane.yawController.rate_pid.kP(yaw_rate_pid_kp);
    plane.yawController.rate_pid.kI(yaw_rate_pid_ki);
    plane.yawController.rate_pid.kD(yaw_rate_pid_kd);
    plane.yawController.rate_pid.ff(yaw_rate_pid_kff);
    plane.yawController.rate_pid.imax(yaw_rate_pid_kimax);
    plane.yawController.rate_pid.filt_T_hz(yaw_rate_pid_filt_T_hz);
    plane.yawController.rate_pid.filt_E_hz(yaw_rate_pid_filt_E_hz);
    plane.yawController.rate_pid.filt_D_hz(yaw_rate_pid_filt_D_hz);
    plane.yawController.rate_pid.slew_limit(yaw_rate_pid_slew_rate_max);
}

void Attack::change_base_pid()
{
    //set first pid
    plane.pitchController.rate_pid.kP(base_pitch_rate_pid._kp);
    plane.pitchController.rate_pid.kI(base_pitch_rate_pid._ki);
    plane.pitchController.rate_pid.kD(base_pitch_rate_pid._kd);
    plane.pitchController.rate_pid.ff(base_pitch_rate_pid._kff);
    plane.pitchController.rate_pid.imax(base_pitch_rate_pid._kimax);
    plane.pitchController.rate_pid.filt_T_hz(base_pitch_rate_pid._filt_T_hz);
    plane.pitchController.rate_pid.filt_E_hz(base_pitch_rate_pid._filt_E_hz);
    plane.pitchController.rate_pid.filt_D_hz(base_pitch_rate_pid._filt_D_hz);
    plane.pitchController.rate_pid.slew_limit(base_pitch_rate_pid._slew_rate_max);

    plane.rollController.rate_pid.kP(base_roll_rate_pid._kp);
    plane.rollController.rate_pid.kI(base_roll_rate_pid._ki);
    plane.rollController.rate_pid.kD(base_roll_rate_pid._kd);
    plane.rollController.rate_pid.ff(base_roll_rate_pid._kff);
    plane.rollController.rate_pid.imax(base_roll_rate_pid._kimax);
    plane.rollController.rate_pid.filt_T_hz(base_roll_rate_pid._filt_T_hz);
    plane.rollController.rate_pid.filt_E_hz(base_roll_rate_pid._filt_E_hz);
    plane.rollController.rate_pid.filt_D_hz(base_roll_rate_pid._filt_D_hz);
    plane.rollController.rate_pid.slew_limit(base_roll_rate_pid._slew_rate_max);

    plane.yawController.rate_pid.kP(base_yaw_rate_pid._kp);
    plane.yawController.rate_pid.kI(base_yaw_rate_pid._ki);
    plane.yawController.rate_pid.kD(base_yaw_rate_pid._kd);
    plane.yawController.rate_pid.ff(base_yaw_rate_pid._kff);
    plane.yawController.rate_pid.imax(base_yaw_rate_pid._kimax);
    plane.yawController.rate_pid.filt_T_hz(base_yaw_rate_pid._filt_T_hz);
    plane.yawController.rate_pid.filt_E_hz(base_yaw_rate_pid._filt_E_hz);
    plane.yawController.rate_pid.filt_D_hz(base_yaw_rate_pid._filt_D_hz);
    plane.yawController.rate_pid.slew_limit(base_yaw_rate_pid._slew_rate_max);
}

void Attack::unlock_limit()
{
    //backup settings
    backup_tecs.flybywire_climb_rate = plane.g.flybywire_climb_rate;
    backup_tecs.pitch_limit_max_cd = plane.aparm.pitch_limit_max_cd;
    backup_tecs.pitch_limit_min_cd = plane.aparm.pitch_limit_min_cd;
    backup_tecs.roll_limit_cd = plane.aparm.roll_limit_cd;
    backup_tecs.airspeed_min = plane.aparm.airspeed_min;
    backup_tecs.airspeed_max = plane.aparm.airspeed_max;

    backup_tecs.TECS_maxClimbRate = plane.TECS_controller._maxClimbRate;
    backup_tecs.TECS_timeConst = plane.TECS_controller._timeConst;
    backup_tecs.TECS_maxSinkRate = plane.TECS_controller._maxSinkRate;
    backup_tecs.TECS_pitch_max = plane.TECS_controller._pitch_max;
    backup_tecs.TECS_pitch_min = plane.TECS_controller._pitch_min;

    backup_L1._L1_period = plane.L1_controller.get_L1_period();
    
    //unlock limit
    plane.g.flybywire_climb_rate = flybywire_climb_rate;
    plane.aparm.pitch_limit_max_cd = pitch_limit_max_cd;
    plane.aparm.pitch_limit_min_cd = pitch_limit_min_cd;
    plane.aparm.roll_limit_cd = roll_limit_cd;
    plane.aparm.airspeed_min = airspeed_min;
    plane.aparm.airspeed_max = airspeed_max;

    plane.TECS_controller._maxClimbRate = TECS_maxClimbRate;
    plane.TECS_controller._timeConst = TECS_timeConst;
    plane.TECS_controller._maxSinkRate = TECS_maxSinkRate;
    plane.TECS_controller._pitch_max = TECS_pitch_max;
    plane.TECS_controller._pitch_min = TECS_pitch_min;

    plane.L1_controller.set_L1_period(_L1_period);
}

void Attack::lock_limit()
{
    //set from backup settings
    plane.g.flybywire_climb_rate = backup_tecs.flybywire_climb_rate;
    plane.aparm.pitch_limit_max_cd = backup_tecs.pitch_limit_max_cd;
    plane.aparm.pitch_limit_min_cd = backup_tecs.pitch_limit_min_cd;
    plane.aparm.roll_limit_cd = backup_tecs.roll_limit_cd;
    plane.aparm.airspeed_min = backup_tecs.airspeed_min;
    plane.aparm.airspeed_max = backup_tecs.airspeed_max;

    plane.TECS_controller._maxClimbRate = backup_tecs.TECS_maxClimbRate;
    plane.TECS_controller._timeConst = backup_tecs.TECS_timeConst;
    plane.TECS_controller._maxSinkRate = backup_tecs.TECS_maxSinkRate;
    plane.TECS_controller._pitch_max = backup_tecs.TECS_pitch_max;
    plane.TECS_controller._pitch_min = backup_tecs.TECS_pitch_min;

    plane.L1_controller.set_L1_period(backup_L1._L1_period);
}

void Attack::backup()
{
    //backup first pid
    base_pitch_rate_pid._kp = plane.pitchController.rate_pid.kP();
    base_pitch_rate_pid._ki = plane.pitchController.rate_pid.kI();
    base_pitch_rate_pid._kd = plane.pitchController.rate_pid.kD();
    base_pitch_rate_pid._kff = plane.pitchController.rate_pid.ff();
    base_pitch_rate_pid._kimax = plane.pitchController.rate_pid.kIMAX();
    base_pitch_rate_pid._filt_T_hz = plane.pitchController.rate_pid.filt_T_hz();
    base_pitch_rate_pid._filt_E_hz = plane.pitchController.rate_pid.filt_E_hz();
    base_pitch_rate_pid._filt_D_hz = plane.pitchController.rate_pid.filt_D_hz();
    base_pitch_rate_pid._slew_rate_max = plane.pitchController.rate_pid.slew_limit();

    base_roll_rate_pid._kp = plane.rollController.rate_pid.kP();
    base_roll_rate_pid._ki = plane.rollController.rate_pid.kI();
    base_roll_rate_pid._kd = plane.rollController.rate_pid.kD();
    base_roll_rate_pid._kff = plane.rollController.rate_pid.ff();
    base_roll_rate_pid._kimax = plane.rollController.rate_pid.kIMAX();
    base_roll_rate_pid._filt_T_hz = plane.rollController.rate_pid.filt_T_hz();
    base_roll_rate_pid._filt_E_hz = plane.rollController.rate_pid.filt_E_hz();
    base_roll_rate_pid._filt_D_hz = plane.rollController.rate_pid.filt_D_hz();
    base_roll_rate_pid._slew_rate_max = plane.rollController.rate_pid.slew_limit();

    base_yaw_rate_pid._kp = plane.yawController.rate_pid.kP();
    base_yaw_rate_pid._ki = plane.yawController.rate_pid.kI();
    base_yaw_rate_pid._kd = plane.yawController.rate_pid.kD();
    base_yaw_rate_pid._kff = plane.yawController.rate_pid.ff();
    base_yaw_rate_pid._kimax = plane.yawController.rate_pid.kIMAX();
    base_yaw_rate_pid._filt_T_hz = plane.yawController.rate_pid.filt_T_hz();
    base_yaw_rate_pid._filt_E_hz = plane.yawController.rate_pid.filt_E_hz();
    base_yaw_rate_pid._filt_D_hz = plane.yawController.rate_pid.filt_D_hz();
    base_yaw_rate_pid._slew_rate_max = plane.yawController.rate_pid.slew_limit();

    //backup settings
    backup_tecs.flybywire_climb_rate = plane.g.flybywire_climb_rate;
    backup_tecs.pitch_limit_max_cd = plane.aparm.pitch_limit_max_cd;
    backup_tecs.pitch_limit_min_cd = plane.aparm.pitch_limit_min_cd;
    backup_tecs.roll_limit_cd = plane.aparm.roll_limit_cd;
    backup_tecs.airspeed_min = plane.aparm.airspeed_min;
    backup_tecs.airspeed_max = plane.aparm.airspeed_max;

    backup_tecs.TECS_maxClimbRate = plane.TECS_controller._maxClimbRate;
    backup_tecs.TECS_timeConst = plane.TECS_controller._timeConst;
    backup_tecs.TECS_maxSinkRate = plane.TECS_controller._maxSinkRate;
    backup_tecs.TECS_pitch_max = plane.TECS_controller._pitch_max;
    backup_tecs.TECS_pitch_min = plane.TECS_controller._pitch_min;

    backup_L1._L1_period = plane.L1_controller.get_L1_period();
}