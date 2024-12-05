
#include "LD2450.h"
#include "esphome/core/log.h"

namespace esphome::ld2450 {

    static const char *TAG = "LD2450";

    void LD2450::setup() {
        for (int i = targets_.size(); i < 3; i++) {
            Target *new_target = new Target();
            targets_.push_back(new_target);
        }

        for (int i = 0; i < targets_.size(); i++) {
            Target *target = targets_[i];
            if (target->get_name() == nullptr) {
                std::string name = std::string("Target ").append(std::to_string(i + 1));
                char *cstr = new char[name.length() + 1];
                std::strcpy(cstr, name.c_str());
                target->set_name(cstr);
            }
            target->set_fast_off_detection(fast_off_detection_);
        }

#ifdef USE_BINARY_SENSOR
        if (occupancy_binary_sensor_ != nullptr)
            occupancy_binary_sensor_->publish_initial_state(false);
#endif
#ifdef USE_BUTTON
        if (restart_button_ != nullptr)
            restart_button_->add_on_press_callback([this]() { this->perform_restart(); });
        if (factory_reset_button_ != nullptr)
            factory_reset_button_->add_on_press_callback([this]() { this->perform_factory_reset(); });
#endif

        read_switch_states();
    }

    void LD2450::dump_config() {
        ESP_LOGCONFIG(TAG, "LD2450 Hub: %s", name_);
        ESP_LOGCONFIG(TAG, "  fast_off_detection: %s", fast_off_detection_ ? "True" : "False");
        ESP_LOGCONFIG(TAG, "  flip_x_axis: %s", flip_x_axis_ ? "True" : "False");
        ESP_LOGCONFIG(TAG, "  max_detection_tilt_angle: %.2f °", max_detection_tilt_angle_);
        ESP_LOGCONFIG(TAG, "  min_detection_tilt_angle: %.2f °", min_detection_tilt_angle_);
        ESP_LOGCONFIG(TAG, "  max_detection_distance: %i mm", max_detection_distance_);
        ESP_LOGCONFIG(TAG, "  max_distance_margin: %i mm", max_distance_margin_);
        ESP_LOGCONFIG(TAG, "  tilt_angle_margin: %.2f °", tilt_angle_margin_);
        log_sensor_version();
        log_bluetooth_mac();
    }

    void LD2450::set_zone(uint8_t zone_index, const std::vector<std::pair<float, float>> &new_polygon) {
        if (zone_index >= this->zones_.size()) {
            ESP_LOGW(TAG, "Zone index %d out of bounds", zone_index);
            return;
        }
        if (new_polygon.size() < 3) {
            ESP_LOGW(TAG, "Polygon must have at least 3 points");
            return;
        }

        ESP_LOGI(TAG, "Updating Zone %d dynamically", zone_index);
        this->zones_[zone_index]->polygon = new_polygon;

        this->send_zone_configuration(zone_index);
    }

    void LD2450::send_zone_configuration(uint8_t zone_index) {
        if (zone_index >= this->zones_.size()) {
            ESP_LOGW(TAG, "Zone index %d out of bounds", zone_index);
            return;
        }

        const auto &zone = this->zones_[zone_index];
        const auto &polygon = zone->polygon;

        std::vector<uint8_t> uart_data;

        uart_data.push_back(0xFD);
        uart_data.push_back(0xFC);
        uart_data.push_back(zone_index);
        uart_data.push_back(polygon.size());

        for (const auto &point : polygon) {
            int16_t x = static_cast<int16_t>(point.first * 100);
            int16_t y = static_cast<int16_t>(point.second * 100);
            uart_data.push_back((x >> 8) & 0xFF);
            uart_data.push_back(x & 0xFF);
            uart_data.push_back((y >> 8) & 0xFF);
            uart_data.push_back(y & 0xFF);
        }

        this->uart_->write_array(uart_data);
        this->uart_->flush();

        ESP_LOGI(TAG, "Zone %d configuration sent via UART", zone_index);
    }
} // namespace esphome::ld2450
