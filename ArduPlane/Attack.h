#pragma once

#include <AP_Param/AP_Param.h>
#include <APM_Control/APM_Control.h>

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
    void change_base_pid();
    void change_extra_pid();


private:

    bool state;

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

    //PID base full wing backup
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
    } base_pitch_rate_pid;

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
};