/**
 * マイコン１個につき１ODrive説が高い
 */

#pragma once

#include "f446_odrive_base.hpp"

namespace osw_no_heya{



class f446_odrive_uart_ascii : public f446_odrive_base
{
public:
    f446_odrive_uart_ascii(PinName _tx, PinName _rx, odrive_settings _settings)
        : odrive_uart(_tx, _rx)
    {
        settings_ = _settings;
        this->init();
    }

    void init() final{
        uint8_t data[2] = {ODriveEnums::AxisState::AXIS_STATE_FULL_CALIBRATION_SEQUENCE, 0};
        this->send_data(data, 2);

        t.attach([this]{
            uint8_t data_closedloop[2] = {ODriveEnums::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL, 0};
            this->send_data(data_closedloop, 2);
        }, std::chrono::milliseconds(2500));
    }

    void reboot() final{
        
    }

private:
    Ticker t;
    UnbufferedSerial odrive_uart;

    void send_data(uint8_t* _data, int _size) final{
        this->odrive_uart.write(_data, _size);
    }
}; // class f446_uart_ascii

class f446_odrive_uart_native_fibre : public f446_odrive_base
{
public:
    f446_odrive_uart_ascii(/* args */);
    
    void 

}; // class f446_uart_ascii

} // namespace osw_no_heya
