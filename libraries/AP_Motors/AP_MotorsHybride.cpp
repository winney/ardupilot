
/*
 *       AP_MotorsHybride.cpp - ArduCopter motors library
 *       Code by Stanislav Punegov. ADUC.de
 *
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsHybride.h"

extern const AP_HAL::HAL &hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsHybride::var_info_hybr[] = {
 
  // @Param: HYBR_CH_IN
  // @DisplayName: Hybride channel in.
  // @Description: Hybride ICE throttle channel in / manual mixing gain channel in.
  // @Range: -1 16
  // @Increment: 1
  // @User: Advanced
   AP_GROUPINFO("CH_IN", 0, AP_MotorsHybride, _hybride_ice_ch_in, MOTORSHYBRIDE_HYBR_CH_IN_DEFAULT),

  // @Param: HYBR_MIX_MODE
  // @DisplayName: Hybride mixing mode.
  // @Description: Hybride mixing mode selection: 0 - ICE throttle pass-through; 1 - constant mixing gain; 2 - PID mixing gain.
  // @Range: 0 2
  // @Increment: 1
  // @User: Advanced
   AP_GROUPINFO("MIX_MODE", 1, AP_MotorsHybride, _hybride_mixing_mode, MOTORSHYBRIDE_HYBR_MIX_MODE_DEFAULT),

  // @Param: HYBR_ICE_RATE
  // @DisplayName: Hybride ICE throttle change max rate.
  // @Description: Hybride ICE throttle changes immidiatly if 0, throttle change from max to min takes 10 sec if value 10.
  // @Range: 0 10
  // @Increment: 0.1
  // @User: Advanced
   AP_GROUPINFO("ICE_RATE", 2, AP_MotorsHybride, _hybride_ice_slew_rate, MOTORSHYBRIDE_HYBR_ICE_RATE_DEFAULT),

   // @Param: HYBR_P_GAIN
   // @DisplayName: Hybride mixing gain.
   // @Description: Hybride mixing gain for constant mixing mode and P-gain for PID-mixing mode.
   // @Range: -2 2
   // @Increment: 0.001
   // @User: Advanced
   AP_GROUPINFO("P_GAIN", 3, AP_MotorsHybride, _hybride_mixing_gain_P, MOTORSHYBRIDE_HYBR_P_GAIN_DEFAULT),

   // @Param: HYBR_I_GAIN
   // @DisplayName: Hybride mixing gain.
   // @Description: Hybride mixing I-gain for PID-mixing mode.
   // @Range: -0.5 0.5
   // @Increment: 0.001
   // @User: Advanced
   AP_GROUPINFO("I_GAIN", 4, AP_MotorsHybride, _hybride_mixing_gain_I, MOTORSHYBRIDE_HYBR_I_GAIN_DEFAULT),

   // @Param: HYBR_D_GAIN
   // @DisplayName: Hybride mixing gain.
   // @Description: Hybride mixing D-gain for PID-mixing mode.
   // @Range: -0.5 0.5
   // @Increment: 0.001
   // @User: Advanced
   AP_GROUPINFO("D_GAIN", 5, AP_MotorsHybride, _hybride_mixing_gain_D, MOTORSHYBRIDE_HYBR_D_GAIN_DEFAULT),

   // @Param: HYBR_I_LIM
   // @DisplayName: Hybride mixing gain.
   // @Description: Hybride mixing I-gain for PID-mixing mode.
   // @Range: -0.5 0.5
   // @Increment: 0.001
   // @User: Advanced
   AP_GROUPINFO("I_LIM", 6, AP_MotorsHybride, _hybride_mixing_I_lim, MOTORSHYBRIDE_HYBR_I_LIM_DEFAULT),

   AP_GROUPEND
};

// init
void AP_MotorsHybride::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    
    
    // record requested frame class and type
    _last_frame_class = frame_class;
    _last_frame_type = frame_type;

    // setup the motors
    setup_motors(frame_type);

    switch (frame_type){
    case MOTOR_FRAME_TYPE_PLUS:
       gcs().send_text(MAV_SEVERITY_ERROR, "HYBRID HEXA-PLUS frame selected.");
       break;
    case MOTOR_FRAME_TYPE_X:
        gcs().send_text(MAV_SEVERITY_ERROR, "HYBRID HEXA-X frame selected.");
        break;
    case MOTOR_FRAME_TYPE_H:
        // H is same as X except middle motors are closer to center
        gcs().send_text(MAV_SEVERITY_ERROR, "HYBRID HEXA-H frame selected.");
        break;
    case MOTOR_FRAME_TYPE_CW_X:
        gcs().send_text(MAV_SEVERITY_ERROR, "HYBRID HEXA-CW-X frame selected.");
        break;
    case MOTOR_FRAME_TYPE_SIDETHRUSTER:
        gcs().send_text(MAV_SEVERITY_ERROR, "Quad with side thrusters frame selected.");
    break; 
    default:
        gcs().send_text(MAV_SEVERITY_ERROR, "Unsupported FRAME_TYPE selected.");
        break;
    }

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);

    // Find the ICE control servo with SERVOXX_Function == 70
    uint8_t chan;
    if (!SRV_Channels::find_channel(SRV_Channel::k_throttle,chan))
    {
        gcs().send_text(MAV_SEVERITY_ERROR, "ICE servo chennal not found");
    }
    else
    {
        /* code */
        gcs().send_text(MAV_SEVERITY_ALERT, "ICE servo chennal #%d",chan+1);

        _ice_servo = SRV_Channels::get_channel_for(SRV_Channel::k_throttle, chan);
    }
}

// set update rate to motors - a value in hertz
void AP_MotorsHybride::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    uint16_t mask = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            mask |= 1U << i;
        }
    }
    rc_set_freq(mask, _speed_hz);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsHybride::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // exit immediately if armed or no change
    if (armed() || (frame_class == _last_frame_class && _last_frame_type == frame_type))
    {
        return;
    }
    _last_frame_class = frame_class;
    _last_frame_type = frame_type;

    // setup the motors
    setup_motors(frame_type);

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

void AP_MotorsHybride::output_to_motors()
{
    int8_t i;

    switch (_spool_state)
    {
    case SpoolState::SHUT_DOWN:
    {
        // no output
        for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
        {
            if (motor_enabled[i])
            {
                _actuator[i] = 0.0f;
            }
        }
        break;
    }
    case SpoolState::GROUND_IDLE:
        // sends output to motors when armed but not flying
        for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
        {
            if (motor_enabled[i])
            {
                set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
            }
        }
        break;
    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        // set motor output based on thrust requests
        for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
        {
            if (motor_enabled[i])
            {
                set_actuator_with_slew(_actuator[i], thrust_to_actuator(_thrust_rpyt_out[i]));
            }
        }
        break;
    }

    // convert output to PWM and send to each motor
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            rc_write(i, output_to_pwm(_actuator[i]));
        }
    }
   
     rc_write_ice(ice_throttle_slew_rate_check(_ice_throttle));
   
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHybride::get_motor_mask()
{
    uint16_t motor_mask = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            motor_mask |= 1U << i;
        }
    }
    uint16_t mask = rc_map_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsHybride::output_armed_stabilizing()
{
    uint8_t i;                      // general purpose counter
    float roll_thrust;              // roll thrust input value, +/- 1.0
    float pitch_thrust;             // pitch thrust input value, +/- 1.0
    float yaw_thrust;               // yaw thrust input value, +/- 1.0
    float throttle_thrust;          // throttle thrust input value, 0.0 - 1.0
    float throttle_avg_max;         // throttle thrust average maximum value, 0.0 - 1.0
    float throttle_thrust_max;      // throttle thrust maximum value, 0.0 - 1.0
    float throttle_thrust_best_rpy; // throttle providing maximum roll, pitch and yaw range without climbing
    float rpy_scale = 1.0f;         // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float yaw_allowed = 1.0f;       // amount of yaw we can fit in
    float thr_adj;                  // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain(); // compensation for battery voltage and altitude
    
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    
    throttle_thrust = get_throttle() * compensation_gain;
    
    throttle_avg_max = _throttle_avg_max * compensation_gain;

    // If thrust boost is active then do not limit maximum thrust
    throttle_thrust_max = _thrust_boost_ratio + (1.0f - _thrust_boost_ratio) * _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f)
    {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= throttle_thrust_max)
    {
        throttle_thrust = throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // ensure that throttle_avg_max is between the input throttle and the maximum throttle
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, throttle_thrust_max);

    // calculate the highest allowed average thrust that will provide maximum control range
    throttle_thrust_best_rpy = MIN(0.5f, throttle_avg_max);

    // calculate throttle that gives most possible room for yaw which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // Under the motor lost condition we remove the highest motor output from our calculations and let that motor go greater than 1.0
    // To ensure control and maximum righting performance Hex and Octo have some optimal settings that should be used
    // Y6               : MOT_YAW_HEADROOM = 350, ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.375, ATC_RAT_PIT_IMAX = 0.375, ATC_RAT_YAW_IMAX = 0.375
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.75,  ATC_RAT_PIT_IMAX = 0.75,  ATC_RAT_YAW_IMAX = 0.375
    // Usable minimums below may result in attitude offsets when motors are lost. Hex aircraft are only marginal and must be handles with care
    // Hex              : MOT_YAW_HEADROOM = 0,   ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.25,  ATC_RAT_PIT_IMAX = 0.25,  ATC_RAT_YAW_IMAX = 0.25
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.5,   ATC_RAT_PIT_IMAX = 0.5,   ATC_RAT_YAW_IMAX = 0.25
    // Quads cannot make use of motor loss handling because it doesn't have enough degrees of freedom.

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller
    float rp_low = 1.0f;   // lowest thrust value
    float rp_high = -1.0f; // highest thrust value
    
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            // calculate the thrust outputs for roll and pitch
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
            // record lowest roll + pitch command
            if (_thrust_rpyt_out[i] < rp_low)
            {
                rp_low = _thrust_rpyt_out[i];
            }
            // record highest roll + pitch command
            if (_thrust_rpyt_out[i] > rp_high && (!_thrust_boost || i != _motor_lost_index))
            {
                rp_high = _thrust_rpyt_out[i];
            }

            // Check the maximum yaw control that can be used on this channel
            // Exclude any lost motors if thrust boost is enabled
            if (!is_zero(_yaw_factor[i]) && (!_thrust_boost || i != _motor_lost_index))
            {
                if (is_positive(yaw_thrust * _yaw_factor[i]))
                {
                    yaw_allowed = MIN(yaw_allowed, fabsf(MAX(1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]), 0.0f) / _yaw_factor[i]));
                }
                else
                {
                    yaw_allowed = MIN(yaw_allowed, fabsf(MAX(throttle_thrust_best_rpy + _thrust_rpyt_out[i], 0.0f) / _yaw_factor[i]));
                }
            }
        }
    }

    // calculate the maximum yaw control that can be used
    // todo: make _yaw_headroom 0 to 1
    float yaw_allowed_min = (float)_yaw_headroom / 1000.0f;

    // increase yaw headroom to 50% if thrust boost enabled
    yaw_allowed_min = _thrust_boost_ratio * 0.5f + (1.0f - _thrust_boost_ratio) * yaw_allowed_min;

    // Let yaw access minimum amount of head room
    yaw_allowed = MAX(yaw_allowed, yaw_allowed_min);

    // Include the lost motor scaled by _thrust_boost_ratio to smoothly transition this motor in and out of the calculation
    if (_thrust_boost && motor_enabled[_motor_lost_index])
    {
        // record highest roll + pitch command
        if (_thrust_rpyt_out[_motor_lost_index] > rp_high)
        {
            rp_high = _thrust_boost_ratio * rp_high + (1.0f - _thrust_boost_ratio) * _thrust_rpyt_out[_motor_lost_index];
        }

        // Check the maximum yaw control that can be used on this channel
        // Exclude any lost motors if thrust boost is enabled
        if (!is_zero(_yaw_factor[_motor_lost_index]))
        {
            if (is_positive(yaw_thrust * _yaw_factor[_motor_lost_index]))
            {
                yaw_allowed = _thrust_boost_ratio * yaw_allowed + (1.0f - _thrust_boost_ratio) * MIN(yaw_allowed, fabsf(MAX(1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[_motor_lost_index]), 0.0f) / _yaw_factor[_motor_lost_index]));
            }
            else
            {
                yaw_allowed = _thrust_boost_ratio * yaw_allowed + (1.0f - _thrust_boost_ratio) * MIN(yaw_allowed, fabsf(MAX(throttle_thrust_best_rpy + _thrust_rpyt_out[_motor_lost_index], 0.0f) / _yaw_factor[_motor_lost_index]));
            }
        }
    }

    if (fabsf(yaw_thrust) > yaw_allowed)
    {
        // not all commanded yaw can be used
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // add yaw control to thrust outputs
    float rpy_low = 1.0f;   // lowest thrust value
    float rpy_high = -1.0f; // highest thrust value
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

            // record lowest roll + pitch + yaw command
            if (_thrust_rpyt_out[i] < rpy_low)
            {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll + pitch + yaw command
            // Exclude any lost motors if thrust boost is enabled
            if (_thrust_rpyt_out[i] > rpy_high && (!_thrust_boost || i != _motor_lost_index))
            {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }
    // Include the lost motor scaled by _thrust_boost_ratio to smoothly transition this motor in and out of the calculation
    if (_thrust_boost)
    {
        // record highest roll + pitch + yaw command
        if (_thrust_rpyt_out[_motor_lost_index] > rpy_high && motor_enabled[_motor_lost_index])
        {
            rpy_high = _thrust_boost_ratio * rpy_high + (1.0f - _thrust_boost_ratio) * _thrust_rpyt_out[_motor_lost_index];
        }
    }

    // calculate any scaling needed to make the combined thrust outputs fit within the output range
    if (rpy_high - rpy_low > 1.0f)
    {
        rpy_scale = 1.0f / (rpy_high - rpy_low);
    }
    if (throttle_avg_max + rpy_low < 0)
    {
        rpy_scale = MIN(rpy_scale, -throttle_avg_max / rpy_low);
    }

    // calculate how close the motors can come to the desired throttle
    rpy_high *= rpy_scale;
    rpy_low *= rpy_scale;
    throttle_thrust_best_rpy = -rpy_low;
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f)
    {
        // Full range is being used by roll, pitch, and yaw.
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        if (thr_adj > 0.0f)
        {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    }
    else
    {
        if (thr_adj < 0.0f)
        {
            // Throttle can't be reduced to desired value
            // todo: add lower limit flag and ensure it is handled correctly in altitude controller
            thr_adj = 0.0f;
        }
        else if (thr_adj > 1.0f - (throttle_thrust_best_rpy + rpy_high))
        {
            // Throttle can't be increased to desired value
            thr_adj = 1.0f - (throttle_thrust_best_rpy + rpy_high);
            limit.throttle_upper = true;
        }
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            _thrust_rpyt_out[i] = throttle_thrust_best_rpy + thr_adj + (rpy_scale * _thrust_rpyt_out[i]);
        }
    }

    // determine throttle thrust for harmonic notch
    const float throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
    // compensation_gain can never be zero
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

    // check for failed motor
    check_for_failed_motor(throttle_thrust_best_plus_adj);

    switch (get_param_mix_mode())
    {
        case HYBRYDE_MIXING_MODE_PASSTHROUGH:
            _ice_throttle = get_ice_rc_in(get_param_ch_in());
        break;
        case HYBRYDE_MIXING_MODE_CONST_GAIN:
            // if RC_IN disabled: _ice_throttle = get_throttle() * HYBRIDE_P_GAIN
            // else: _ice_throttle = get_throttle() * HYBRIDE_P_GAIN * RC_IN_throttle

            _ice_throttle = get_throttle()*get_param_mix_P_gain();  //get_param_mix_P_gain
            if (get_param_ch_in() >=0)                              //if rc_in enabled
            {
                _ice_throttle *= get_ice_rc_in(get_param_ch_in());
            }
        break;
        case HYBRIDE_MIXING_MODE_PID:
            _ice_throttle = ice_pid_control(get_ice_rc_in(get_param_ch_in()) - get_throttle());
        break;
        default:
            static uint16_t counter = 0;
            if (++counter > 500)
            {
                counter = 0;
                gcs().send_text(MAV_SEVERITY_ERROR, "Param HYBRIDE_MIX_MODE not set");
            }
    }
}

// check for failed motor
//   should be run immediately after output_armed_stabilizing
//   first argument is the sum of:
//      a) throttle_thrust_best_rpy : throttle level (from 0 to 1) providing maximum roll, pitch and yaw range without climbing
//      b) thr_adj: the difference between the pilot's desired throttle and throttle_thrust_best_rpy
//   records filtered motor output values in _thrust_rpyt_out_filt array
//   sets thrust_balanced to true if motors are balanced, false if a motor failure is detected
//   sets _motor_lost_index to index of failed motor
void AP_MotorsHybride::check_for_failed_motor(float throttle_thrust_best_plus_adj)
{
    // record filtered and scaled thrust output for motor loss monitoring purposes
    float alpha = 1.0f / (1.0f + _loop_rate * 0.5f);
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            _thrust_rpyt_out_filt[i] += alpha * (_thrust_rpyt_out[i] - _thrust_rpyt_out_filt[i]);
        }
    }

    float rpyt_high = 0.0f;
    float rpyt_sum = 0.0f;
    uint8_t number_motors = 0.0f;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            number_motors += 1;
            rpyt_sum += _thrust_rpyt_out_filt[i];
            // record highest filtered thrust command
            if (_thrust_rpyt_out_filt[i] > rpyt_high)
            {
                rpyt_high = _thrust_rpyt_out_filt[i];
                // hold motor lost index constant while thrust boost is active
                if (!_thrust_boost)
                {
                    _motor_lost_index = i;
                }
            }
        }
    }

    float thrust_balance = 1.0f;
    if (rpyt_sum > 0.1f)
    {
        thrust_balance = rpyt_high * number_motors / rpyt_sum;
    }
    // ensure thrust balance does not activate for multirotors with less than 6 motors
    if (number_motors >= 6 && thrust_balance >= 1.5f && _thrust_balanced)
    {
        _thrust_balanced = false;
    }
    if (thrust_balance <= 1.25f && !_thrust_balanced)
    {
        _thrust_balanced = true;
    }

    // check to see if thrust boost is using more throttle than _throttle_thrust_max
    if ((_throttle_thrust_max * get_compensation_gain() > throttle_thrust_best_plus_adj) && (rpyt_high < 0.9f) && _thrust_balanced)
    {
        _thrust_boost = false;
    }
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHybride::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed())
    {
        return;
    }

    // loop through all the possible orders spinning any motors that match that description
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i] && _test_order[i] == motor_seq)
        {
            // turn on this motor
            rc_write(i, pwm);
        }
    }
}

// output_test_num - spin a motor connected to the specified output channel
//  (should only be performed during testing)
//  If a motor output channel is remapped, the mapped channel is used.
//  Returns true if motor output is set, false otherwise
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
bool AP_MotorsHybride::output_test_num(uint8_t output_channel, int16_t pwm)
{
    if (!armed())
    {
        return false;
    }

    // Is channel in supported range?
    if (output_channel > AP_MOTORS_MAX_NUM_MOTORS - 1)
    {
        return false;
    }

    // Is motor enabled?
    if (!motor_enabled[output_channel])
    {
        return false;
    }

    rc_write(output_channel, pwm); // output

    
    return true;
}

// add_motor
void AP_MotorsHybride::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS)
    {

        // increment number of motors if this motor is being newly motor_enabled
        if (!motor_enabled[motor_num])
        {
            motor_enabled[motor_num] = true;
        }

        // set roll, pitch, thottle factors and opposite motor (for stability patch)
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // call parent class method
        add_motor_num(motor_num);
    }
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void AP_MotorsHybride::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void AP_MotorsHybride::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsHybride::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS)
    {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _roll_factor[motor_num] = 0;
        _pitch_factor[motor_num] = 0;
        _yaw_factor[motor_num] = 0;
    }
}

void AP_MotorsHybride::setup_motors(motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        remove_motor(i);
    }

    bool success = true;

    switch (frame_type){
    case MOTOR_FRAME_TYPE_PLUS:
        add_motor(AP_MOTORS_MOT_1, 0, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1);
        add_motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_3, -120, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5);
        add_motor(AP_MOTORS_MOT_4, 60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
        add_motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
        break;
    case MOTOR_FRAME_TYPE_X:
        add_motor(AP_MOTORS_MOT_1, 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
        add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
        add_motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6);
        add_motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_5, 30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
        add_motor(AP_MOTORS_MOT_6, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4);
        break;
    case MOTOR_FRAME_TYPE_H:
        // H is same as X except middle motors are closer to center
        add_motor_raw(AP_MOTORS_MOT_1, -1.0f, 0.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
        add_motor_raw(AP_MOTORS_MOT_2, 1.0f, 0.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
        add_motor_raw(AP_MOTORS_MOT_3, 1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6);
        add_motor_raw(AP_MOTORS_MOT_4, -1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor_raw(AP_MOTORS_MOT_5, -1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
        add_motor_raw(AP_MOTORS_MOT_6, 1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4);
        break;
    case MOTOR_FRAME_TYPE_CW_X:
        add_motor(AP_MOTORS_MOT_1, 30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
        add_motor(AP_MOTORS_MOT_2, 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
        add_motor(AP_MOTORS_MOT_3, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_4, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4);
        add_motor(AP_MOTORS_MOT_5, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
        add_motor(AP_MOTORS_MOT_6, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6);
        break;
    case MOTOR_FRAME_TYPE_SIDETHRUSTER:
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
        add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
    break; 
    default:
        // Flyworks Hybride frame class does not support this frame type
        success = false;
        break;
    }

//    add_motor_raw(AP_MOTORS_MOT_7, 0.0f, 0.0f, 0.0f, 7);
    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    _flags.initialised_ok = success;
}

// normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
void AP_MotorsHybride::normalise_rpy_factors()
{
    float roll_fac = 0.0f;
    float pitch_fac = 0.0f;
    float yaw_fac = 0.0f;

    // find maximum roll, pitch and yaw factors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            if (roll_fac < fabsf(_roll_factor[i]))
            {
                roll_fac = fabsf(_roll_factor[i]);
            }
            if (pitch_fac < fabsf(_pitch_factor[i]))
            {
                pitch_fac = fabsf(_pitch_factor[i]);
            }
            if (yaw_fac < fabsf(_yaw_factor[i]))
            {
                yaw_fac = fabsf(_yaw_factor[i]);
            }
        }
    }

    // scale factors back to -0.5 to +0.5 for each axis
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_enabled[i])
        {
            if (!is_zero(roll_fac))
            {
                _roll_factor[i] = 0.5f * _roll_factor[i] / roll_fac;
            }
            if (!is_zero(pitch_fac))
            {
                _pitch_factor[i] = 0.5f * _pitch_factor[i] / pitch_fac;
            }
            if (!is_zero(yaw_fac))
            {
                _yaw_factor[i] = 0.5f * _yaw_factor[i] / yaw_fac;
            }
        }
    }
}
/*
  call vehicle supplied thrust compensation if set. This allows
  vehicle code to compensate for vehicle specific motor arrangements
  such as tiltrotors or tiltwings
*/
void AP_MotorsHybride::thrust_compensation(void)
{
    if (_thrust_compensation_callback){
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
    }
}

/*
  returns relative desired throttle in range 0..1, where 0 == RC#_MIN and 1 == RC#_MAX
  This function ignore RC#_REV parameter
*/
 double AP_MotorsHybride::get_ice_rc_in(int8_t ch_in)
{
    uint16_t val= 1500;
    double out = 0;
    
    if (ch_in < 0) return 0;

    RC_Channel *c = rc().channel((uint8_t)ch_in - 1);
 
    if (c != nullptr) 
    {
        // get ICE control channel
        val = c-> get_radio_in();

        if (val < c->get_radio_min()) return 0;
        else if (val > c->get_radio_max()) return 1;

        val -= c->get_radio_min();
        out = (double)val / (c->get_radio_max() - c->get_radio_min());
    }
    return out;
}

int8_t AP_MotorsHybride::get_param_ch_in(void)
{
    char name[] = "HYBR_CH_IN";
    float val;
    
    if (!AP_Param::get(name,val))   return -1;  //error level
    if (val < 0)                    return -1;  //error level
    if (val > 16)                   return -1;  //error level
    return ((int8_t)val);
}

int8_t AP_MotorsHybride::get_param_mix_mode(void)
{
    char name[] = "HYBR_MIX_MODE";
    float val;
    
    if (!AP_Param::get(name,val))   return -1;  //error level
    if (val < 1)                    return -1;  //error level
    if (val > 5)                    return -1;  //error level
    return ((int8_t)val);
}

double  AP_MotorsHybride::get_param_mix_P_gain(void)
{
    char name[] = "HYBR_P_GAIN";
    float val;

    if (!AP_Param::get(name,val))   return 0;   //error level
    if (val < 0)                    return 0;   //min allowed level
    if (val > 2)                    return 2;   //max allowed level
    return val;
}

double  AP_MotorsHybride::get_param_mix_I_gain(void)
{
    char name[] = "HYBR_I_GAIN";
    float val;

    if (!AP_Param::get(name,val))   return 0;   //error level
    if (val < 0)                    return 0;   //min allowed level
    if (val > 2)                    return 2;   //max allowed level
    return val;
}

double  AP_MotorsHybride::get_param_mix_D_gain(void)
{
    char name[] = "HYBR_D_GAIN";
    float val;

    if (!AP_Param::get(name,val))   return 0;   //error level
    if (val < 0)                    return 0;   //min allowed level
    if (val > 10)                   return 10;   //max allowed level
    return val;
}

double  AP_MotorsHybride::get_param_mix_I_lim(void)
{
    char name[] = "HYBR_I_LIM";
    float val;

    if (!AP_Param::get(name,val))   return 0;   //error level
    if (val < 0)                    return 0;   //min allowed level
    if (val > 2)                    return 2;   //max allowed level
    return val;
}

double  AP_MotorsHybride::get_param_ice_slew_rate(void)
{
    char name[] = "HYBR_ICE_RATE";
    float val;

    if (!AP_Param::get(name,val))   return 0;   //error level
    if (val < 0)                    return 0;   //min allowed level
    if (val > 10)                   return 10;  //max allowed level
    return val;
}

/*
    Limit the ICE throttle rate level
    inpute/output range 0..1
    Function returns updated throttle level
*/
double AP_MotorsHybride::ice_throttle_slew_rate_check(double thr)
{
    static double thr_last = 0;
    double max_diff = 1/((get_param_ice_slew_rate() * _loop_rate)+1);

    if(thr >= thr_last)
    {
        if ((thr - thr_last) > max_diff) thr_last += max_diff;
        else  thr_last = thr; 
    }
    else 
    {
        if ((thr_last - thr) > max_diff) thr_last -= max_diff;
        else  thr_last = thr;
    }

    return thr_last;
} 

/*
    Set ICE servo pwm to the value, equel to calculated ICE throttle.
    This function ignore SERVO#_REV parameter
*/
void AP_MotorsHybride::rc_write_ice(double thr)
{    
    uint16_t ice_throttle_to_set = _ice_servo->get_output_min();
    
    ice_throttle_to_set += (uint16_t)(thr * (_ice_servo->get_output_max() - _ice_servo->get_output_min()));
    
    _ice_servo->set_output_pwm(ice_throttle_to_set);
}

/*
    PID controler
*/
double AP_MotorsHybride::ice_pid_control(double err)
{    
    double val;
    static double integral = 0;
    static double last_err = 0;

    integral += err;
    if (integral> get_param_mix_I_lim())integral = get_param_mix_I_lim();
    else if (integral< - get_param_mix_I_lim())integral = - get_param_mix_I_lim();

    val = err * get_param_mix_P_gain() + integral * get_param_mix_I_gain() + (err - last_err) * get_param_mix_D_gain();
    if (val>1) val = 1;
    else if (val<0) val = 0;

    return val;
}