
#include "LD2450.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2450 {

static const char *TAG = "LD2450";

void LD2450::setup() {
    // Setup logic...
}

void LD2450::loop() {
    // Loop logic...
}

void LD2450::dump_config() {
    // Dump config logic...
}

void LD2450::send_zone_configuration(uint8_t zone_index) {
    // Validate the zone index
    if (zone_index >= this->zones_.size()) {
        ESP_LOGW(TAG, "Zone index %d out of bounds", zone_index);
        return;
    }

    // Retrieve the zone and its polygon
    const auto &zone = this->zones_[zone_index];
    const auto &polygon = zone->polygon; // Correct pointer access

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
        int16_t x = static_cast<int16_t>(point.first * 100);  // Convert meters to cm
        int16_t y = static_cast<int16_t>(point.second * 100); // Convert meters to cm
        uart_data.push_back((x >> 8) & 0xFF);                 // High byte of X
        uart_data.push_back(x & 0xFF);                        // Low byte of X
        uart_data.push_back((y >> 8) & 0xFF);                 // High byte of Y
        uart_data.push_back(y & 0xFF);                        // Low byte of Y
    }

    // Send the command via UART
    this->uart_->write_array(uart_data);
    this->uart_->flush();

    ESP_LOGI(TAG, "Zone %d configuration sent via UART", zone_index);
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
    this->zones_[zone_index]->polygon = new_polygon; // Use '->' for pointer access

    // Send the updated zone configuration via UART
    this->send_zone_configuration(zone_index);
}

// Other functions...

} // namespace ld2450
} // namespace esphome
