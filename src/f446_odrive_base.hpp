/**
 * @file f446_odrive_base.hpp
 * @brief base class for f446-odrive libs
 * one axis - one object
 */

#pragma once

#include <mbed.h>
#include <functional>
#include <chrono>
#include <memory>

#include "ODriveEnums.h"

// reference to https://docs.odriverobotics.com/v/0.5.4/can-protocol.html#messages
typedef struct
{
    uint8_t axis_id;
    uint8_t requested_state;
    uint8_t startup_config;
    uint8_t controll_mode;
    uint8_t input_mode;
    float vel_limit;
    float current_limit;
    float traj_vel_limit;
    float traj_accel_limit;
    float traj_inertia;
    float set_pos_count;
    float set_pos_gain;
    float set_vel_gain;
} odrive_settings;

// dimension depends on ctrl mode
typedef float ODriveInput;

//
typedef struct{
    float encoder_estimates;
    float iq;
}odrive_data;

typedef struct
{
    uint32_t heart_beat;
    uint64_t motor_error;
    uint32_t encoder_error;
    uint32_t sensorless_error;
    float encoder_estimates;
    float iq;
}odrive_fulldata;

namespace osw_no_heya{

class f446_odrive_base
{
public:    
    /// @brief send settings
    virtual void init() = 0;

    /// @brief get data from odrive
    /// @return return odrive_data via pointer
    virtual void get_data(odrive_data* _data) = 0;
    virtual void get_data(odrive_fulldata* _data) = 0;

    virtual void save_config() = 0;
    virtual void erace_config() = 0;

    virtual void reboot() = 0;
    virtual void clear_error() = 0;

protected:
    odrive_settings settings_;
    odrive_data odrive_data_;
    // std::function<void(odrive_data)> read_func_;

    bool is_closed_loop_;

    uint32_t heartbeat_msg1, heartbeat_msg2;
    uint8_t heartbeat_msg3;
    float iq1, iq2;
    virtual void odrive_callback() = 0;

    virtual void send_data(uint32_t* _data, int _size) = 0;

}; // class f446_odrive_base

} // namespace osw_no_heya
