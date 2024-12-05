
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

namespace esphome::ld2450
{

    class LD2450 : public uart::UARTDevice, public Component
    {
    public:
        void setup() override;
        void loop() override;
        void send_zone_configuration(uint8_t zone_index);
        void set_zone(uint8_t zone_index, const std::vector<std::pair<float, float>> &new_polygon);

    protected:
        void process_message(uint8_t *msg, int len);
        void process_config_message(uint8_t *msg, int len);
    };

} // namespace esphome::ld2450
