#include "LD2450.h"
#include "esphome/core/log.h"

namespace esphome::ld2450 {

static const char *TAG = "LD2450";

void LD2450::setup() {
    // Fill target list with mock targets if not present
    for (int i = targets_.size(); i < 3; i++) {
        Target *new_target = new Target();
        targets_.push_back(new_target);
    }

    for (int i = 0; i < targets_.size(); i++) {
        Target *target = targets_[i];
        // Generate Names if not present
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
    // Acquire current switch states and update related components
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
#ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "OccupancyBinarySensor", occupancy_binary_sensor_);
#endif
#ifdef USE_NUMBER
    LOG_NUMBER("  ", "MaxTiltAngleNumber", max_angle_number_);
    LOG_NUMBER("  ", "MinTiltAngleNumber", min_angle_number_);
    LOG_NUMBER("  ", "MaxDistanceNumber", max_distance_number_);
#endif
#ifdef USE_BUTTON
    LOG_BUTTON("  ", "RestartButton", restart_button_);
    LOG_BUTTON("  ", "FactoryResetButton", factory_reset_button_);
#endif
    LOG_SWITCH("  ", "TrackingModeSwitch", tracking_mode_switch_);
    LOG_SWITCH("  ", "BluetoothSwitch", bluetooth_switch_);
    LOG_SELECT("  ", "BaudRateSelect", baud_rate_select_);
    ESP_LOGCONFIG(TAG, "Zones:");
    for (Zone *zone : zones_) {
        zone->dump_config();
    }

    // Read and log Firmware-version
    log_sensor_version();
    log_bluetooth_mac();
}

void LD2450::set_zone(uint8_t zone_index, const std::vector<std::pair<float, float>> &new_polygon) {
    // Validate the input
    if (zone_index >= this->zones_.size()) {
        ESP_LOGW(TAG, "Zone index %d out of bounds", zone_index);
        return;
    }
    if (new_polygon.size() < 3) {
        ESP_LOGW(TAG, "Polygon must have at least 3 points");
        return;
    }

    // Update the polygon for the specified zone
    ESP_LOGI(TAG, "Updating Zone %d dynamically", zone_index);
    this->zones_[zone_index]->polygon = new_polygon;

    // Send the updated zone configuration via UART
    this->send_zone_configuration(zone_index);
}

void LD2450::send_zone_configuration(uint8_t zone_index) {
    // Validate the zone index
    if (zone_index >= this->zones_.size()) {
        ESP_LOGW(TAG, "Zone index %d out of bounds", zone_index);
        return;
    }

    // Retrieve the zone and its polygon
    const auto &zone = this->zones_[zone_index];
    const auto &polygon = zone->polygon;

    // Prepare UART data for zone configuration
    std::vector<uint8_t> uart_data;

    // Command header
    uart_data.push_back(0xFD); // Start byte
    uart_data.push_back(0xFC); // Command type

    // Zone index
    uart_data.push_back(zone_index);

    // Number of points in the polygon
    uart_data.push_back(polygon.size());

    // Add the points to the UART data
    for (const auto &point : polygon) {
        int16_t x = static_cast<int16_t>(point.first * 100); // Convert meters to cm
        int16_t y = static_cast<int16_t>(point.second * 100); // Convert meters to cm
        uart_data.push_back((x >> 8) & 0xFF); // High byte of X
        uart_data.push_back(x & 0xFF);        // Low byte of X
        uart_data.push_back((y >> 8) & 0xFF); // High byte of Y
        uart_data.push_back(y & 0xFF);        // Low byte of Y
    }

    // Send the command via UART
    this->uart_->write_array(uart_data);
    this->uart_->flush();

    ESP_LOGI(TAG, "Zone %d configuration sent via UART", zone_index);
}

void LD2450::loop() {
    // Add loop implementation here as in your original code
    // Ensuring this does not interfere with updated zone handling
}

void LD2450::process_message(uint8_t *msg, int len) {
    // Add process_message implementation here
    // Including updates to dynamic zone handling
}

void LD2450::process_config_message(uint8_t *msg, int len) {
    // Add process_config_message implementation here
}

void LD2450::log_sensor_version() {
    const uint8_t read_version[2] = {COMMAND_READ_VERSION, 0x00};
    send_config_message(read_version, 2);
}

void LD2450::log_bluetooth_mac() {
    const uint8_t read_mac[4] = {COMMAND_READ_MAC, 0x00, 0x01, 0x00};
    send_config_message(read_mac, 4);
}

void LD2450::perform_restart() {
    const uint8_t restart[2] = {COMMAND_RESTART, 0x00};
    send_config_message(restart, 2);
    read_switch_states();
}

void LD2450::perform_factory_reset() {
    const uint8_t reset[2] = {COMMAND_FACTORY_RESET, 0x00};
    send_config_message(reset, 2);
    perform_restart();
}

void LD2450::set_tracking_mode(bool mode) {
    const uint8_t set_tracking_mode[2] = {mode ? COMMAND_MULTI_TRACKING_MODE : COMMAND_SINGLE_TRACKING_MODE, 0x00};
    send_config_message(set_tracking_mode, 2);

    const uint8_t request_tracking_mode[2] = {COMMAND_READ_TRACKING_MODE, 0x00};
    send_config_message(request_tracking_mode, 2);
}

void LD2450::set_bluetooth_state(bool state) {
    const uint8_t set_bt[4] = {COMMAND_BLUETOOTH, 0x00, state, 0x00};
    send_config_message(set_bt, 4);
    perform_restart();
}

void LD2450::read_switch_states() {
    const uint8_t request_tracking_mode[2] = {COMMAND_READ_TRACKING_MODE, 0x00};
    send_config_message(request_tracking_mode, 2);
    log_bluetooth_mac();
}

void LD2450::set_baud_rate(BaudRate baud_rate) {
    const uint8_t set_baud_rate[4] = {COMMAND_SET_BAUD_RATE, 0x00, baud_rate, 0x00};
    send_config_message(set_baud_rate, 4);
    perform_restart();
}

void LD2450::write_command(uint8_t *msg, int len) {
    // Write frame header
    write_array({0xFD, 0xFC, 0xFB, 0xFA});

    // Write message length
    write(static_cast<uint8_t>(len));
    write(static_cast<uint8_t>(len >> 8));

    // Write message content
    write_array(msg, len);

    // Write frame end
    write_array({0x04, 0x03, 0x02, 0x01});

    flush();
}

} // namespace esphome::ld2450
