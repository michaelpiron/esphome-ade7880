#include "ade7880.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ade7880 {

static const char *TAG = "ade7880";

void ADE7880::dump_config() {
  ESP_LOGCONFIG(TAG, "ADE7880:");
  if (this->has_irq_) {
    ESP_LOGCONFIG(TAG, "  IRQ Pin: GPIO%u", this->irq_pin_number_);
  }
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Voltage A Sensor", this->voltage_a_sensor_);
  LOG_SENSOR("  ", "Voltage B Sensor", this->voltage_b_sensor_);
  LOG_SENSOR("  ", "Voltage C Sensor", this->voltage_c_sensor_);
  LOG_SENSOR("  ", "Current A Sensor", this->current_a_sensor_);
  LOG_SENSOR("  ", "Current B Sensor", this->current_b_sensor_);
  LOG_SENSOR("  ", "Current C Sensor", this->current_c_sensor_);
  LOG_SENSOR("  ", "Active Power A Sensor", this->active_power_a_sensor_);
  LOG_SENSOR("  ", "Active Power B Sensor", this->active_power_b_sensor_);
  LOG_SENSOR("  ", "Active Power C Sensor", this->active_power_c_sensor_);
}

#define ADE_PUBLISH_(name, factor) \
  if (name && this->name##_sensor_) { \
    float value = *name / factor; \
    this->name##_sensor_->publish_state(value); \
  }
#define ADE_PUBLISH(name, factor) ADE_PUBLISH_(name, factor)

void ADE7880::update() {
  if (!this->is_setup_)
    return;
/* Done until this point /**/
  auto active_power_a = this->ade_read_<int32_t>(0x0312);
  ADE_PUBLISH(active_power_a, 154.0f);
  auto active_power_b = this->ade_read_<int32_t>(0x0313);
  ADE_PUBLISH(active_power_b, 154.0f);
  auto current_a = this->ade_read_<uint32_t>(0x031A);
  ADE_PUBLISH(current_a, 100000.0f);
  auto current_b = this->ade_read_<uint32_t>(0x031B);
  ADE_PUBLISH(current_b, 100000.0f);
  auto voltage = this->ade_read_<uint32_t>(0x031C);
  ADE_PUBLISH(voltage, 26000.0f);

  //    auto apparent_power_a = this->ade_read_<int32_t>(0x0310);
  //    auto apparent_power_b = this->ade_read_<int32_t>(0x0311);
  //    auto reactive_power_a = this->ade_read_<int32_t>(0x0314);
  //    auto reactive_power_b = this->ade_read_<int32_t>(0x0315);
  //    auto power_factor_a = this->ade_read_<int16_t>(0x010A);
  //    auto power_factor_b = this->ade_read_<int16_t>(0x010B);
}

}  // namespace ade7880
}  // namespace esphome
