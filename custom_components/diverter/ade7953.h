#pragma once
 
 #include "esphome/core/component.h"
 #include "esphome/core/hal.h"
 #include "esphome/components/i2c/i2c.h"
 #include "esphome/components/sensor/sensor.h"
 
 #include <vector>
 
 namespace esphome {
 namespace ade7953 {

 static const int32_t COUNT_10J = 128;            //=10*207024/4862401*power_factor
 
 class ADE7953 : public i2c::I2CDevice, public PollingComponent {
  public:
   void set_irq_pin(InternalGPIOPin *irq_pin) { irq_pin_ = irq_pin; }
   void set_load_pin(InternalGPIOPin *load_pin) { load_pin_ = load_pin; }
   void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
   void set_current_a_sensor(sensor::Sensor *current_a_sensor) { current_a_sensor_ = current_a_sensor; }
   void set_current_b_sensor(sensor::Sensor *current_b_sensor) { current_b_sensor_ = current_b_sensor; }
   void set_active_power_a_sensor(sensor::Sensor *active_power_a_sensor) {
     active_power_a_sensor_ = active_power_a_sensor;
   }
   /// @brief 
   /// @param active_power_b_sensor 
   void set_active_power_b_sensor(sensor::Sensor *active_power_b_sensor) {
     active_power_b_sensor_ = active_power_b_sensor;
   }

  // Below is Diverter specific
  void set_energy_buffer_sensor(sensor::Sensor *energy_buffer_sensor) {
    energy_buffer_sensor_ = energy_buffer_sensor;
  }
  void set_energy_diverted_sensor(sensor::Sensor *energy_diverted_sensor) {
    energy_diverted_sensor_ = energy_diverted_sensor;
  }
  
  void set_diverter_parameters(uint32_t energy_buffer_size, float energy_threshold_high_percent, float energy_threshold_low_percent) {
    energy_buffer_size_=energy_buffer_size*COUNT_10J/10;
    energy_threshold_high_=energy_buffer_size_*energy_threshold_high_percent;
    energy_threshold_low_=energy_buffer_size_*energy_threshold_low_percent;
    if (energy_threshold_low_>energy_threshold_high_) energy_threshold_low_=energy_threshold_high_;
  }
 
   void setup() override {
     if (this->irq_pin_ != nullptr) {
       this->irq_pin_->setup();
     }
     if (this->load_pin_ != nullptr) {            //Diverter
       this->load_pin_->setup();
     }
     this->load_pin_->digital_write(false);       //Diverter
     this->set_timeout(100, [this]() {
       this->ade_write_8_(0x0010, 0x04);          //lock interface, keep HPFEN
       this->ade_write_8_(0x00FE, 0xAD);          //unlock next command
       this->ade_write_16_(0x0120, 0x0030);       //required register setting
       this->ade_write_8_(0x0001, 0x01);          //disable the active power no-load feature
       this->ade_write_8_(0x0004, 0x01);          //Enable ALWATT - active energy line cycle accumulation
       this->ade_write_16_(0x0101, 0x001);        //Accumulate 1 cycle
       this->ade_write_32_(0x032C, 0x00140000);   //Enable CYCEND Interrupt
       this->ade_read_32_(0x032E, 0x000000);      //Reset interrupt status

       this->is_setup_ = true;
     });
   }
 
   void dump_config() override;
 
   void update() override;
 
  protected:
   i2c::ErrorCode ade_write_8_(uint16_t reg, uint8_t value) {
     std::vector<uint8_t> data;
     data.push_back(reg >> 8);
     data.push_back(reg >> 0);
     data.push_back(value);
     return write(data.data(), data.size());
   }
   i2c::ErrorCode ade_write_16_(uint16_t reg, uint16_t value) {
     std::vector<uint8_t> data;
     data.push_back(reg >> 8);
     data.push_back(reg >> 0);
     data.push_back(value >> 8);
     data.push_back(value >> 0);
     return write(data.data(), data.size());
   }
   i2c::ErrorCode ade_write_32_(uint16_t reg, uint32_t value) {
     std::vector<uint8_t> data;
     data.push_back(reg >> 8);
     data.push_back(reg >> 0);
     data.push_back(value >> 24);
     data.push_back(value >> 16);
     data.push_back(value >> 8);
     data.push_back(value >> 0);
     return write(data.data(), data.size());
   }
   i2c::ErrorCode ade_read_32_(uint16_t reg, uint32_t *value) {
     uint8_t reg_data[2];
     reg_data[0] = reg >> 8;
     reg_data[1] = reg >> 0;
     i2c::ErrorCode err = write(reg_data, 2);
     if (err != i2c::ERROR_OK)
       return err;
     uint8_t recv[4];
     err = read(recv, 4);
     if (err != i2c::ERROR_OK)
       return err;
     *value = 0;
     *value |= ((uint32_t) recv[0]) << 24;
     *value |= ((uint32_t) recv[1]) << 16;
     *value |= ((uint32_t) recv[2]) << 8;
     *value |= ((uint32_t) recv[3]);
     return i2c::ERROR_OK;
   }
 

  InternalGPIOPin *irq_pin_{nullptr};
  InternalGPIOPin *load_pin_{nullptr};
  bool  load_status_ = false;

  int32_t energy_buffer_ = 0;
  uint32_t energy_diverted_ = 0;
  uint32_t energy_buffer_size_;
  uint32_t energy_threshold_high_;
  uint32_t energy_threshold_low_;
   bool is_setup_{false};
   sensor::Sensor *voltage_sensor_{nullptr};
   sensor::Sensor *current_a_sensor_{nullptr};
   sensor::Sensor *current_b_sensor_{nullptr};
   sensor::Sensor *active_power_a_sensor_{nullptr};
   sensor::Sensor *active_power_b_sensor_{nullptr};
   sensor::Sensor *energy_buffer_sensor_{nullptr};
   sensor::Sensor *energy_diverted_sensor_{nullptr};
 };
 
 }  // namespace ade7953
 }  // namespace esphome
