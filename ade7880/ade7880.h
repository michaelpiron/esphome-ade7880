#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ade7880 {

class ADE7880 : public i2c::I2CDevice, public PollingComponent {
 public:
  void set_irq_pin(uint8_t irq_pin) {
    has_irq_ = true;
    irq_pin_number_ = irq_pin;
  }
  void set_voltage_a_sensor(sensor::Sensor *voltage_a_sensor) { voltage_a_sensor_ = voltage_a_sensor; }
  void set_voltage_b_sensor(sensor::Sensor *voltage_b_sensor) { voltage_b_sensor_ = voltage_b_sensor; }
  void set_voltage_c_sensor(sensor::Sensor *voltage_c_sensor) { voltage_c_sensor_ = voltage_c_sensor; }
  void set_current_a_sensor(sensor::Sensor *current_a_sensor) { current_a_sensor_ = current_a_sensor; }
  void set_current_b_sensor(sensor::Sensor *current_b_sensor) { current_b_sensor_ = current_b_sensor; }
  void set_current_c_sensor(sensor::Sensor *current_c_sensor) { current_c_sensor_ = current_c_sensor; }
  void set_active_power_a_sensor(sensor::Sensor *active_power_a_sensor) {
    active_power_a_sensor_ = active_power_a_sensor;
  }
  void set_active_power_b_sensor(sensor::Sensor *active_power_b_sensor) {
    active_power_b_sensor_ = active_power_b_sensor;
  }
  void set_active_power_c_sensor(sensor::Sensor *active_power_c_sensor) {
    active_power_c_sensor_ = active_power_c_sensor;
  }
  
  /**
  * @brief   Quickly calculate the log base 2 of a number for the PGA gains
  * */
  static uint32_t quick_log2(uint32_t num)
  {
    uint32_t log = 0;
    while (num >>= 1)
      ++log;
    return log;
  }
  
  
  void setup() override {
    if (this->has_irq_) {
      auto pin = GPIOPin(this->irq_pin_number_, INPUT);
      this->irq_pin_ = &pin;
      this->irq_pin_->setup();
    }
    
    /* Setup procedure ADE7880 according to specification ( https://www.analog.com/media/en/technical-documentation/data-sheets/ADE7880.pdf ) */
    this->set_timeout(100, [this]() {
      // Configure ADE GPIOs
      
      // Reset ADE
      
      // Set Power Mode
      
      // Choose I2C as main interface and lock
      this->ade_write_<uint8_t>(ADE_CONFIG2, 0x02);
      
      // Set gains
      uint16_t gain;
      
      gain =  (quick_log2(ADE_PGA_VGAIN) << 6) | 
              (quick_log2(ADE_PGA_NGAIN) << 3) | 
              (quick_log2(ADE_PGA_IGAIN) << 0);
      
      ade_write_<uint16_t>(ADE_Gain, gain);
      
      // Set CONFIG register
#if ADE_USE_ROGOWSKI
      this->ade_write_<uint32_t>(ADE_DICOEFF, 0xFF8000);
      this->ade_write_<uint16_t>(ADE_CONFIG, 0x0001); /* Not sure about number of bits */
#endif
      // Set COMPMODE register
#if ADE_USE_60HZ
      /* Default value + bit 14 set to 1*/
      this->ade_write_<uint16_t>(ADE_COMPMODE, 0x41FF);
#endif
      
      // Initialize all the other data memory RAM registers
      
      // Initialize the WTHR, VARTHR, VATHR, VLEVEL andVNOM registers
      this->ade_write_<uint8_t>(ADE_WTHR, ADE_WTHR_VAL);
      this->ade_write_<uint8_t>(ADE_VARTHR, ADE_VARTHR_VAL);
      this->ade_write_<uint8_t>(ADE_VATHR, ADE_VATHR_VAL);
      this->ade_write_<uint32_t>(ADE_VLEVEL, ADE_VLEVEL_VAL);
      this->ade_write_<uint32_t>(ADE_VNOM, ADE_VNOM_VAL);
      
      // Initialize CF1DEN, CF2DEN, and CF3DEN
      this->ade_write_<uint16_t>(ADE_CF1DEN, ADE_CFXDEN_VAL);
      this->ade_write_<uint16_t>(ADE_CF2DEN, ADE_CFXDEN_VAL);
      this->ade_write_<uint16_t>(ADE_CF3DEN, ADE_CFXDEN_VAL);
      
      // Enable RAM protection
      this->ade_write_<uint8_t>(0xE7FE, 0xAD);
      this->ade_write_<uint8_t>(0xE7E3, 0x80);
      
      // Read back all data memory RAM registers to ensure thatthey initialized with the desired values.
      
      // Start the DSP
      this->ade_write_<uint16_t>(ADE_RUN, 0x0001);
      
      // Read the energy registers xWATTHR, xVAHR, xFWATTHR, and xFVARHR to erase their content and start energyaccumulation from a known state.
      this->ade_read_<uint32_t>(ADE_AWATTHR);
      this->ade_read_<uint32_t>(ADE_BWATTHR);
      this->ade_read_<uint32_t>(ADE_CWATTHR);
      this->ade_read_<uint32_t>(ADE_AVAHR);
      this->ade_read_<uint32_t>(ADE_BVAHR);
      this->ade_read_<uint32_t>(ADE_CVAHR);
      this->ade_read_<uint32_t>(ADE_AFWATTHR);
      this->ade_read_<uint32_t>(ADE_BFWATTHR);
      this->ade_read_<uint32_t>(ADE_CFWATTHR);
      this->ade_read_<uint32_t>(ADE_AFVARHR);
      this->ade_read_<uint32_t>(ADE_BFVARHR);
      this->ade_read_<uint32_t>(ADE_CFVARHR);
      
      // Enable the CF1, CF2, CF3 frequency convertor outputs
      this->ade_write_<uint16_t>(ADE_CFMODE, 0x08A0); /* To double check */
      
      // Setup done
      this->is_setup_ = true;
    });
  }

  void dump_config() override;

  void update() override;

 protected:
  template<typename T> bool ade_write_(uint16_t reg, T value) {
    std::vector<uint8_t> data;
    data.push_back(reg >> 8);
    data.push_back(reg >> 0);
    for (int i = sizeof(T) - 1; i >= 0; i--)
      data.push_back(value >> (i * 8));
    return this->write_bytes_raw(data);
  }
  template<typename T> optional<T> ade_read_(uint16_t reg) {
    uint8_t hi = reg >> 8;
    uint8_t lo = reg >> 0;
    if (!this->write_bytes_raw({hi, lo}))
      return {};
    auto ret = this->read_bytes_raw<sizeof(T)>();
    if (!ret.has_value())
      return {};
    T result = 0;
    for (int i = 0, j = sizeof(T) - 1; i < sizeof(T); i++, j--)
      result |= T((*ret)[i]) << (j * 8);
    return result;
  }

  bool has_irq_ = false;
  uint8_t irq_pin_number_;
  GPIOPin *irq_pin_{nullptr};
  bool is_setup_{false};
  sensor::Sensor *voltage_a_sensor_{nullptr};
  sensor::Sensor *voltage_b_sensor_{nullptr};
  sensor::Sensor *voltage_c_sensor_{nullptr};
  sensor::Sensor *current_a_sensor_{nullptr};
  sensor::Sensor *current_b_sensor_{nullptr};
  sensor::Sensor *current_c_sensor_{nullptr};
  sensor::Sensor *active_power_a_sensor_{nullptr};
  sensor::Sensor *active_power_b_sensor_{nullptr};
  sensor::Sensor *active_power_c_sensor_{nullptr};
};

}  // namespace ade7880
}  // namespace esphome
