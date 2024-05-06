#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c_bus_arduino.h"
#include "SiC45x.h"

namespace esphome {
namespace sic45x_led {

class SiC45XComponent;

namespace brightness
{
    class SiC45X_Brightness;
    static void set_reference(SiC45X_Brightness *brightness, SiC45XComponent *ic);
} // namespace brightness

class SiC45XComponent : public i2c::ArduinoI2CBus {
 public:
  void setup();
  void dump_config() override;
  float get_setup_priority() const override;
  void update();

  void set_sensor_voltage(sensor::Sensor *voltage){ this->voltage = voltage; }
  void set_sensor_current(sensor::Sensor *current){ this->current = current; }
  void set_sensor_temperature(sensor::Sensor *temperature){ this->temperature = temperature; }
  void set_sensor_power(sensor::Sensor *power){ this->power = power; }
  void set_led_nominal_voltage(float led_nominal_voltage){ this->led_nominal_voltage = led_nominal_voltage; }
  void set_led_nominal_current(float led_nominal_current){ this->led_nominal_current = led_nominal_current; }
  void set_leds_series(uint16_t leds_series){ this->leds_series = leds_series; }
  void set_leds_parallel(uint16_t leds_parallel){ this->leds_parallel = leds_parallel; }
  void set_overvoltage_tol(float overvoltage_tol){ this->overvoltage_tol = overvoltage_tol; }
  void set_overcurrent_tol(float overcurrent_tol){ this->overcurrent_tol = overcurrent_tol; }
  void set_min_input_voltage(float min_input_voltage){ this->min_input_voltage = min_input_voltage; }
  void set_min_enable_voltage(float min_enable_voltage){ this->min_enable_voltage = min_enable_voltage; }
  void set_max_input_voltage(float max_input_voltage){ this->max_input_voltage = max_input_voltage; }
  void set_max_input_current(float max_input_current){ this->max_input_current = max_input_current; }
  void set_max_operating_temp(float max_operating_temp){ this->max_operating_temp = max_operating_temp; }
  void set_output_power_limit_percent(float output_power_limit_percent){ 
    this->output_power_limit_percent = output_power_limit_percent;
  }
  void set_switching_frequency(uint16_t switching_frequency){ this->switching_frequency = switching_frequency; }
  void set_setpoint_low_voltage(float setpoint_low_voltage){ 
    this->setpoint_low_voltage = setpoint_low_voltage; 
  }
  void set_setpoint_high_voltage(float setpoint_high_voltage){
    this->setpoint_high_voltage = setpoint_high_voltage;
  }
  void set_setpoint_low_current_percent(float setpoint_low_current_percent){
    this->setpoint_low_current_percent = setpoint_low_current_percent;
  }
  void set_setpoint_high_current_percent(float setpoint_high_current_percent){
    this->setpoint_high_current_percent = setpoint_high_current_percent;
  }

  void setBrightnessPercent(float brightness);

  void register_brightness(brightness::SiC45X_Brightness *brightness){
    this->brightness_values.push_back(brightness);
    set_reference(brightness, this);
  }
  std::vector<brightness::SiC45X_Brightness *> get_brightness_values(){ return brightness_values; }

  //missing functions from I2CDevice
  void set_i2c_address(uint8_t address) { address_ = address; }
  void set_i2c_bus(I2CBus *bus) { bus_ = bus; }

  SiC45XComponent(){}
  //missing functions from PollingComponent
  //SiC45XComponent(uint32_t update_interval) : update_interval_(update_interval) {}

  void call_setup();
  void start_poller();
  void stop_poller();

  uint32_t get_update_interval() const { return this->update_interval_; }
  void set_update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }
  
 private:
  SiC45x ic;
  uint8_t address_ = 0;
  I2CBus *bus_ = nullptr;
  uint32_t update_interval_;

  //led params
  float led_nominal_voltage = 0.0;
  float led_nominal_current = 0.0;
  uint16_t leds_series = 1;
  uint16_t leds_parallel = 1;
  float led_max_Vout = 0.0;
  float led_max_Iout = 0.0;
  float overvoltage_tol = 0.05;
  float overcurrent_tol = 0.05;
  float output_power_limit_percent = 1.0;
  //driver params
  float switching_frequency = 600;
  float min_input_voltage = 10.8;
  float min_enable_voltage = 10;
  float max_input_voltage = 20; //MAX 20V!!
  float max_input_current = 8; //Depends on schematics and Version e.g. SiC453, SiC451
  float max_operating_temp = 100; //Operating junction temperature: MAX 125°C
  //calibration variables
  float setpoint_low_voltage = -1;
  float setpoint_high_voltage = -1;
  float setpoint_low_current_percent = -1;
  float setpoint_high_current_percent = -1;

  bool use_auto_calibration = false; //does sadly not work!

 protected:
  void setVoltage(float voltage);
  float searchVoltageAtCurrent(float currentMin, float currentMax);

  std::vector<brightness::SiC45X_Brightness *> brightness_values;

  sensor::Sensor *voltage{nullptr};
  sensor::Sensor *current{nullptr};
  sensor::Sensor *temperature{nullptr};
  sensor::Sensor *power{nullptr};
};

}  // namespace sic45x_led
}  // namespace esphome