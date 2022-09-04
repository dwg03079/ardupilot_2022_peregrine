#pragma once

#include <AP_Param/AP_Param.h>
#include <APM_Control/APM_Control.h>
#include <AP_Mission/AP_Mission.h>

//custom - Attack option, Fox 2300mm
class Attack {
public:
    Attack()
        //: Attack()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    Attack(const Attack& other) = delete;
    Attack& operator=(const Attack&) = delete;

    static const struct AP_Param::GroupInfo var_info[];


    void attack_update();
    bool set_servo(uint8_t _channel, uint16_t pwm);
    void check_command(const AP_Mission::Mission_Command& cmd);
    void change_base_pid();
    void change_extra_pid();
    void unlock_limit();
    void lock_limit();

    bool get_state_wp_ignore() { return state_wp_ignore; }
    void set_state_wp_ignore_reset() { state_wp_ignore = false; }

    void backup();

private:

    bool check_backup = false;

    bool state_extra_pid = true;
    bool state_limit = true;
    bool state_wp_ignore = false;

    AP_Int8 rc_channel;

    AP_Int8 servo_01_channel;
    AP_Int16 servo_01_pwm_lock;
    AP_Int16 servo_01_pwm_unlock;
    AP_Int8 servo_02_channel;
    AP_Int16 servo_02_pwm_lock;
    AP_Int16 servo_02_pwm_unlock;

    //PID
        
    AP_Float pitch_rate_pid_kp;// { 0.08 };
    AP_Float pitch_rate_pid_ki;//{ 0.15 };
    AP_Float pitch_rate_pid_kd;//{ 0 };
    AP_Float pitch_rate_pid_kff;//{ 0.345 };
    AP_Float pitch_rate_pid_kimax;//{ 0.666 };
    AP_Float pitch_rate_pid_filt_T_hz;//{ 3 };         // PID target filter frequency in Hz
    AP_Float pitch_rate_pid_filt_E_hz;//{ 0 };         // PID error filter frequency in Hz    
    AP_Float pitch_rate_pid_filt_D_hz;//{ 12 };         // PID derivative filter frequency in Hz
    AP_Float pitch_rate_pid_slew_rate_max;//{ 150 };

    AP_Float roll_rate_pid_kp;//{ 0.04 };
    AP_Float roll_rate_pid_ki;//{ 0.15 };
    AP_Float roll_rate_pid_kd;//{ 0 };
    AP_Float roll_rate_pid_kff;//{ 0.345 };
    AP_Float roll_rate_pid_kimax;//{ 0.666 };
    AP_Float roll_rate_pid_filt_T_hz;//{ 3 };         // PID target filter frequency in Hz
    AP_Float roll_rate_pid_filt_E_hz;//{ 0 };         // PID error filter frequency in Hz
    AP_Float roll_rate_pid_filt_D_hz;//{ 12 };         // PID derivative filter frequency in Hz
    AP_Float roll_rate_pid_slew_rate_max;//{ 150 };

    AP_Float yaw_rate_pid_kp;//{ 0.04 };
    AP_Float yaw_rate_pid_ki;//{ 0.15 };
    AP_Float yaw_rate_pid_kd;//{ 0 };
    AP_Float yaw_rate_pid_kff;//{ 0.15 };
    AP_Float yaw_rate_pid_kimax;//{ 0.666 };
    AP_Float yaw_rate_pid_filt_T_hz;//{ 3 };         // PID target filter frequency in Hz
    AP_Float yaw_rate_pid_filt_E_hz;//{ 0 };         // PID error filter frequency in Hz
    AP_Float yaw_rate_pid_filt_D_hz;//{ 12 };         // PID derivative filter frequency in Hz
    AP_Float yaw_rate_pid_slew_rate_max;//{ 150 };


    AP_Int8 flybywire_climb_rate;   //2.0f
    AP_Int16 pitch_limit_max_cd;    //2000
    AP_Int16 pitch_limit_min_cd;    //-2500
    AP_Int16 roll_limit_cd;         //4500
    AP_Int16 airspeed_min;          //9
    AP_Int16 airspeed_max;          //22

    AP_Float TECS_maxClimbRate;     //5.0f
    AP_Float TECS_timeConst;        //5.0f
    AP_Float TECS_maxSinkRate;      //5.0f
    AP_Int8 TECS_pitch_max;         //15
    AP_Int8 TECS_pitch_min;         //0

    AP_Float _L1_period;



    //PID base full wing backup
    struct {
        float _kp{ 0.04 };
        float _ki{ 0.15 };
        float _kd{ 0 };
        float _kff{ 0.345 };
        float _kimax{ 0.666 };
        float _filt_T_hz{ 3 };         // PID target filter frequency in Hz
        float _filt_E_hz{ 0 };         // PID error filter frequency in Hz
        float _filt_D_hz{ 12 };         // PID derivative filter frequency in Hz
        float _slew_rate_max{ 150 };
    } base_pitch_rate_pid;

    struct {
        float _kp{ 0.08 };
        float _ki{ 0.15 };
        float _kd{ 0 };
        float _kff{ 0.345 };
        float _kimax{ 0.666 };
        float _filt_T_hz{ 3 };         // PID target filter frequency in Hz
        float _filt_E_hz{ 0 };         // PID error filter frequency in Hz
        float _filt_D_hz{ 12 };         // PID derivative filter frequency in Hz
        float _slew_rate_max{ 150 };
    } base_roll_rate_pid;

    struct {
        float _kp{ 0.04 };
        float _ki{ 0.15 };
        float _kd{ 0 };
        float _kff{ 0.15 };
        float _kimax{ 0.666 };
        float _filt_T_hz{ 3 };         // PID target filter frequency in Hz
        float _filt_E_hz{ 0 };         // PID error filter frequency in Hz
        float _filt_D_hz{ 12 };         // PID derivative filter frequency in Hz
        float _slew_rate_max{ 150 };
    } base_yaw_rate_pid;

    struct {
        int8_t flybywire_climb_rate;
        int16_t pitch_limit_max_cd;
        int16_t pitch_limit_min_cd;
        int16_t roll_limit_cd;
        int16_t airspeed_min;
        int16_t airspeed_max;

        float TECS_maxClimbRate;
        float TECS_timeConst;
        float TECS_maxSinkRate;
        int8_t TECS_pitch_max;
        int8_t TECS_pitch_min;
    } backup_tecs;

    struct {
        float _L1_period;
    } backup_L1;
};