
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "target.h"
#include "zone.h"
#include <vector>

namespace esphome::ld2450 {

    class LD2450 : public uart::UARTDevice, public Component {
    public:
        void setup() override;
        void dump_config() override;
        void set_zone(uint8_t zone_index, const std::vector<std::pair<float, float>> &new_polygon);

    private:
        void send_zone_configuration(uint8_t zone_index);

        std::vector<Zone *> zones_;
    };

} // namespace esphome::ld2450
