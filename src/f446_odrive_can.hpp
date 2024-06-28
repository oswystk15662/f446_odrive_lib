/**
 * 各軸に対するCAN id の割り振りはodrivetoolsでやってください
 */

#pragma once

#include "f446_odrive_base.hpp"

namespace osw_no_heya{

constexpr int ODRIVE_CAN_FREQ = 400000;


/// @sa https://docs.odriverobotics.com/v/0.5.4/can-protocol.html
typedef enum{
    CANOPEN_NMT_MESSAGE = 0x00,
    HEART_BEAT_MESSAGE = 0x01,
    ODRIVE_ESTOP_MESSAGE = 0x02,
    GET_MOTOR_ERROR = 0x03,
    GET_ENCODER_ERROR = 0x04,
    GET_SENSERLESS_ERROR = 0x05,
    SET_AXIS_NODE_ID = 0x06,
    SET_AXIS_REQUESTED_STATE = 0x07,
    SET_AXIS_STARTUP_CONFIG = 0x08,
    GET_ENCODER_ESTIMATES = 0x09,
    GET_ENCODER_COUNT = 0x0A,
    SET_CONTROLLER_MODES = 0x0B,
    SET_INPUT_POS = 0x0C,
    SET_INPUT_VEL = 0x0D,
    SET_INPUT_TORQUE = 0x0E,
    SET_LIMITS = 0x0F,
    START_ANTICOGGINNG = 0x10,
    SET_TRAJ_VEL_LIMIT = 0x11,
    SET_TRAJ_ACCEL_LIMIT = 0x12,
    SET_TRAJ_INERTIA = 0x13,
    GET_IQ = 0x14,
    GET_SENSORLESS_ESTIMATE = 0x15,
    ODRIVE_REBOOT = 0x16,
    GET_VBUS_VOLTAGE = 0x17,
    CLEAR_ERRORS = 0x18,
    SET_LINEAR_COUNT = 0x19,
    SET_POSITION_GAIN = 0x1A,
    SET_VEL_GAINS = 0x1B,
    CANOPEN_HEARTBEAT_MESSAGE = 0x700
} ODrive_cmd_ID;

class f446_odrive_can : public f446_odrive_base
{
public:
    f446_odrive_can(PinName _rd, PinName _td, uint32_t _can_id, odrive_settings _settings)
        : odrive_can(_rd, _td)
        , axis_can_id_(_can_id)
        , settings_(_settings)
    {
        odrive_can.frequency(ODRIVE_CAN_FREQ); // set 400kHz
        this->init();
    }

    void init() final{
        uint32_t data[3]; // uint32 じゃないとfloatがコピーできない

        // full calib
        data[0] = this->axis_can_id_ << 5 + ODrive_cmd_ID::SET_AXIS_REQUESTED_STATE;
        data[1] = ODriveEnums::AxisState::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
        data[2] = 0;
        this->send_data(data, 2);

        t.attach([this]{
            uint32_t data_closedloop[2] = {ODriveEnums::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL, 0};
            this->send_data(data_closedloop, 2);
            this->is_closed_loop_ = true;
        }, std::chrono::milliseconds(2500));

        // wait for closed loop
        while(!this->is_closed_loop_){
            ThisThread::sleep_for(10ms);
        }

        // set ctrl mode
        data[0] = this->axis_can_id_ << 5 + ODrive_cmd_ID::SET_CONTROLLER_MODES;
        data[1] = this->settings_.controll_mode;
        data[2] = this->settings_.input_mode;
        this->send_data(data, 3);
        
        // set vel and current limit
        data[0] = this->axis_can_id_ << 5 + ODrive_cmd_ID::SET_LIMITS;
        memcpy(&data[1], &this->settings_.traj_vel_limit, sizeof(this->settings_.traj_vel_limit));
        data[2] = 0;
        this->send_data(data, 2);

        data[0] = this->axis_can_id_ << 5 + ODrive_cmd_ID::SET_TRAJ_ACCEL_LIMIT;
        memcpy(&data[1], &this->settings_.traj_accel_limit, sizeof(this->settings_.traj_accel_limit));
        data[2] = 0;
        this->send_data(data, 2);

        // set gains
        data[0] = this->axis_can_id_ << 5 + ODrive_cmd_ID::SET_POSITION_GAIN;
        memcpy(&data[1], &this->settings_.set_pos_gain, sizeof(this->settings_.set_pos_gain));
        data[2] = 0;
        this->send_data(data, 2);

        data[0] = this->axis_can_id_ << 5 + ODrive_cmd_ID::SET_VEL_GAINS;
        memcpy(&data[1], &this->settings_.set_vel_gain, sizeof(this->settings_.set_vel_gain));
        data[2] = 0;
        this->send_data(data, 2);
    }

    void get_data(odrive_data* _data) final{

    }

    void get_data(odrive_fulldata* _data) final{

    }

    void save_config() final{
        static_assert(1, "CAN can't save config");
    }

    void erace_config() final{
        static_assert(1, "CAN can't erace config");
    }

    void reboot() final{
        uint32_t data[3]; // uint32 じゃないとfloatがコピーできない

        data[0] = this->axis_can_id_ << 5 + ODrive_cmd_ID::ODRIVE_REBOOT;
        data[1] = 0;
        data[2] = 0;
        this->send_data(data, 1);
    }

    void clear_error() final{
        uint32_t data[3]; // uint32 じゃないとfloatがコピーできない

        data[0] = this->axis_can_id_ << 5 + ODrive_cmd_ID::CLEAR_ERRORS;
        data[1] = 0;
        data[2] = 0;
        this->send_data(data, 1);
    }

private:

    Ticker t;
    
    CAN odrive_can;
    uint32_t axis_can_id_;
    odrive_settings settings_;

    void odrive_callback() final{
        CANMessage heartbeat_msg(this->axis_can_id_ << 5 + ODrive_cmd_ID::CANOPEN_HEARTBEAT_MESSAGE);
        CANMessage iq_msg(this->axis_can_id_ << 5 + ODrive_cmd_ID::GET_IQ);

        // iq のほうが頻度高いので
        if (odrive_can.read(iq_msg))
        {
            this->iq1 = iq_msg.data[0];
            this->iq2 = iq_msg.data[1];
        }
        else if(odrive_can.read(heartbeat_msg)){
            // データが壊れてたら、uint8_t -> floatする必要あり
            this->heartbeat_msg1 = heartbeat_msg.data[0];
            this->heartbeat_msg2 = heartbeat_msg.data[1];
            this->heartbeat_msg3 = heartbeat_msg.data[2];
        }
        else{
            // 通信が死んでると思われるので、idleにする。
            // printfでマイコン落としてもいいけどしぶそうなので
            uint32_t data[3];
            data[0] = this->axis_can_id_ << 5 + ODrive_cmd_ID::SET_AXIS_REQUESTED_STATE;
            data[1] = ODriveEnums::AxisState::AXIS_STATE_IDLE;
            data[2] = 0;
            this->send_data(data, 2);
        }
    }

    void send_data(uint32_t* _data, int _size) final{
        size_t data_size = (_size - 1) * 4;
        uint8_t sending_data[data_size];
        memcpy(sending_data, &_data[1], data_size);
        CANMessage msg(_data[0], sending_data, data_size);
        this->odrive_can.write(msg);
    }
}; // class f446_odrive_can

} // namespace osw_no_heya