
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#include "target.h"
#include "zone.h"
#include "tracking_mode_switch.h"
#include "bluetooth_switch.h"
#include "baud_rate_select.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif

#define SENSOR_UNAVAILABLE_TIMEOUT 4000
#define CONFIG_RECOVERY_INTERVAL 60000
#define POST_RESTART_LOCKOUT_DELAY 2000

#define COMMAND_MAX_RETRIES 10
#define COMMAND_RETRY_DELAY 100

#define COMMAND_ENTER_CONFIG 0xFF
#define COMMAND_LEAVE_CONFIG 0xFE
#define COMMAND_READ_VERSION 0xA0
#define COMMAND_RESTART 0xA3
#define COMMAND_FACTORY_RESET 0xA2

#define COMMAND_READ_TRACKING_MODE 0x91
#define COMMAND_SINGLE_TRACKING_MODE 0x80
#define COMMAND_MULTI_TRACKING_MODE 0x90

#define COMMAND_READ_MAC 0xA5
#define COMMAND_BLUETOOTH 0xA4

#define COMMAND_SET_BAUD_RATE 0xA1

namespace esphome::ld2450
{

    enum BaudRate
    {
        BAUD_9600 = 0x01,
        BAUD_19200 = 0x02,
        BAUD_38400 = 0x03,
        BAUD_57600 = 0x04,
        BAUD_115200 = 0x05,
        BAUD_230400 = 0x06,
        BAUD_256000 = 0x07,
        BAUD_460800 = 0x08,
    };

    static const std::map<std::string, BaudRate> BAUD_STRING_TO_ENUM{
        {"9600", BAUD_9600},
        {"19200", BAUD_19200},
        {"38400", BAUD_38400},
        {"57600", BAUD_57600},
        {"115200", BAUD_115200},
        {"230400", BAUD_230400},
        {"256000", BAUD_256000},
        {"460800", BAUD_460800}};

    class TrackingModeSwitch;
    class BluetoothSwitch;
    class BaudRateSelect;

    class EmptyButton : public button::Button
    {
    protected:
        virtual void press_action() {};
    };

    class LD2450 : public uart::UARTDevice, public Component
    {
#ifdef USE_BINARY_SENSOR
        SUB_BINARY_SENSOR(occupancy)
#endif
#ifdef USE_SENSOR
        SUB_SENSOR(target_count)
#endif
#ifdef USE_NUMBER
        SUB_NUMBER(max_distance)
        SUB_NUMBER(max_angle)
        SUB_NUMBER(min_angle)
#endif
#ifdef USE_BUTTON
        SUB_BUTTON(restart)
        SUB_BUTTON(factory_reset)
#endif
    public:
        void setup() override;
        void loop() override;
        void dump_config() override;
        void set_zone(uint8_t zone_index, const std::vector<std::pair<float, float>> &new_polygon);
        void log_sensor_version();
        void log_bluetooth_mac();
        void perform_restart();
        void perform_factory_reset();
        void set_tracking_mode(bool mode);
        void set_bluetooth_state(bool state);
        void read_switch_states();
        void set_baud_rate(BaudRate baud_rate);

    private:
        void send_zone_configuration(uint8_t zone_index);

    protected:
        void process_message(uint8_t *msg, int len);
        void process_config_message(uint8_t *msg, int len);
        void write_command(uint8_t *msg, int len);
        void send_config_message(const uint8_t *msg, int len)
        {
            command_queue_.push_back(std::vector<uint8_t>(msg, msg + len));
        }

        uint8_t peek_status_ = 0;
        const char *name_ = "LD2450";
        bool flip_x_axis_ = false;
        bool is_occupied_ = false;
        bool fast_off_detection_ = false;
        bool configuration_mode_ = false;
        bool is_applying_changes_ = false;
        bool sensor_available_ = false;
        int configuration_message_length_ = 0;
        uint32_t command_last_sent_ = 0;
        uint32_t last_message_received_ = 0;
        uint32_t last_available_change_ = 0;
        uint32_t last_config_leave_attempt_ = 0;
        uint32_t apply_change_lockout_ = 0;
        int last_available_size_ = 0;
        std::vector<std::vector<uint8_t>> command_queue_;
        int command_send_retries_ = 0;
        float max_detection_tilt_angle_ = 90;
        float min_detection_tilt_angle_ = -90;
        float tilt_angle_margin_ = 5;
        int16_t max_detection_distance_ = 6000;
        int16_t max_distance_margin_ = 250;
        std::vector<Target *> targets_;
        std::vector<Zone *> zones_;
        TrackingModeSwitch *tracking_mode_switch_ = nullptr;
        BluetoothSwitch *bluetooth_switch_ = nullptr;
        BaudRateSelect *baud_rate_select_ = nullptr;
    };
} // namespace esphome::ld2450
