#include "sic45x_led.h"

#include "esphome/core/log.h"

namespace esphome {
namespace sic45x_led {

static const char *const TAG = "sic45x_led";

void SiC45XComponent::update() {
    if(this->voltage != nullptr)
        this->voltage->publish_state(ic.getReadVout());
    if(this->current != nullptr)
        this->current->publish_state(ic.getReadIout());
    if(this->temperature != nullptr)
        this->temperature->publish_state(ic.getReadTemperature());
    if(this->power != nullptr)
        this->power->publish_state(ic.getReadPout());
    this->status_clear_warning();
}


void SiC45XComponent::setup() {
    //define maximum values
    this->led_max_Iout = this->led_nominal_current * this->leds_parallel;
    this->led_max_Vout = this->led_nominal_voltage * this->leds_series;

    ic.begin(address_, (*(reinterpret_cast<ArduinoI2CBus*>(bus_)->getWire())));

    ic.sendClearFaults();
    ESP_LOGD(TAG, "%s", ic.printStatusWord());

    //disable output
    ic.setOperation(
        SIC45X_OPERATION_ON_OFF_DISABLED
        | SIC45X_OPERATION_OFF_B_IMMEDIATE
        | SIC45X_OPERATION_MARGIN_COMMAND
        | SIC45X_OPERATION_MRGNFLT_FOLLOW
    );

    ic.setFrequencySwitch(this->switching_frequency); //set switching freq
    ic.setInterleave(SIC45X_INTERLEAVE_MODE_MASTER); //set working mode to master

    //OVER VOLTAGE PROTECTION
    ic.setVoutOvFaultLimit(this->led_max_Vout + this->overvoltage_tol); //set overvoltage fault limit
    ic.setVoutOvWarnLimit(this->led_max_Vout); //set overvoltage warn limit
    ic.setVoutOvFaultResponse(
        SIC45X_VOUT_OV_FAULT_RESPONSE_OVRSP_DISABLED_WHILE_FAULTY
        | SIC45X_VOUT_UV_FAULT_RESPONSE_UVRTY_NO_RESTART
        | SIC45X_VOUT_UV_FAULT_RESPONSE_UVDLY_NO_DELAY
    ); //Disable Output while faulty, do not restart

    //UNDER VOLTAGE PROTECTION
    ic.setVoutUvWarnLimit(0); //undervoltage ignored
    ic.setVoutUvFaultLimit(0);
    ic.setVoutUvFaultResponse(
        SIC45X_VOUT_UV_FAULT_RESPONSE_UVRSP_CONTINUE 
        | SIC45X_VOUT_UV_FAULT_RESPONSE_UVRTY_NO_RESTART
        | SIC45X_VOUT_UV_FAULT_RESPONSE_UVDLY_NO_DELAY
    );

    //OVER CURRENT PROTECTION
    ic.setIoutOcFaultLimit(this->led_max_Iout + this->overcurrent_tol);
    ic.setIoutOcWarnLimit(this->led_max_Iout);
    ic.setIoutOcFaultResponse(
        SIC45X_IOUT_OC_FAULT_RESPONSE_OCRSP_DISABLED_WHILE_FAULTY
        | SIC45X_IOUT_OC_FAULT_RESPONSE_OCRTY_NO_RESTART
        | SIC45X_IOUT_OC_FAULT_RESPONSE_OCDLY_NO_DELAY
    ); //Disable Output while faulty, do not restart
    
    //INPUT PROTECTION
    ic.setVinOn(this->min_enable_voltage); 
    ic.setVinOff(this->min_enable_voltage - 0.5); 
    ic.setVinOvFaultLimit(this->max_input_voltage);
    ic.setVinUvWarnLimit(this->min_input_voltage);
    ic.setIinOcWarnLimit(this->max_input_current);
    ic.setVinOvFaultResponse(
        SIC45X_VIN_OV_FAULT_RESPONSE_OVRSP_DISABLED_WHILE_FAULTY
        | SIC45X_VIN_OV_FAULT_RESPONSE_OVRTY_NO_RESTART
        | SIC45X_VIN_OV_FAULT_RESPONSE_OVDLY_NO_DELAY
    ); //Disable Output while faulty, do not restart

    //TEMPERATURE PROTECTION
    ic.setOtWarnLimit(this->max_operating_temp - 10);
    ic.setOtFaultLimit(this->max_operating_temp);
    ic.setOtFaultResponse(
        SIC45X_VIN_OV_FAULT_RESPONSE_OVRSP_DISABLED_WHILE_FAULTY
        | SIC45X_VIN_OV_FAULT_RESPONSE_OVRTY_NO_RESTART
        | SIC45X_VIN_OV_FAULT_RESPONSE_OVDLY_NO_DELAY
    ); //Disable Output while faulty, do not restart

    //On Off Config
    ic.setOnOffConfiguration(
        SIC45X_ON_OFF_CONFIGURATION_PU_COMMAND
        | SIC45X_ON_OFF_CONFIGURATION_CMD_RESPOND
        | SIC45X_ON_OFF_CONFIGURATION_EN_IGNORE
        | SIC45X_ON_OFF_CONFIGURATION_ENPOL_LOW
        | SIC45X_ON_OFF_CONFIGURATION_OFFB1_IMMEDIATE
    ); // On listen to Bus Commands
    
    ESP_LOGD(TAG, "%s", ic.printStatusWord());
    
    //Enable ic
    ic.setOperation(
        SIC45X_OPERATION_ON_OFF_ENABLED
        | SIC45X_OPERATION_OFF_B_IMMEDIATE
        | SIC45X_OPERATION_MARGIN_COMMAND
        | SIC45X_OPERATION_MRGNFLT_FOLLOW
    );

    if(use_auto_calibration){
        //Does not work :(
        this->setpoint_low_current_percent = .3;
        this->setpoint_high_current_percent = .7;
        this->setpoint_low_voltage = 
            searchVoltageAtCurrent((this->setpoint_low_current_percent - this->overcurrent_tol) * this->led_max_Iout, 
                                    this->setpoint_low_current_percent * this->led_max_Iout);
        this->setpoint_high_voltage = 
            searchVoltageAtCurrent((this->setpoint_high_current_percent - this->overcurrent_tol) * this->led_max_Iout,
                                    this->setpoint_high_current_percent * this->led_max_Iout);
    }else if(this->setpoint_low_voltage != -1 &&
             this->setpoint_high_voltage != -1 &&
             this->setpoint_low_current_percent != -1 &&
             this->setpoint_high_current_percent != -1){
        //Use manual calibration (two point)
        //All values set via config yaml!
    }else if(this->setpoint_low_voltage != -1){
        //Use manual calibration (single point) (point vout (0%) to 100% vout)
        this->setpoint_high_current_percent = 1;
        this->setpoint_low_current_percent = 0;
        //this->setpoint_low_voltage is set from yaml
        this->setpoint_high_voltage = this->led_max_Vout * .99;
    }else{
        //Use no calibration
        this->setpoint_low_current_percent = .3;
        this->setpoint_high_current_percent = .7;
        this->setpoint_low_voltage = this->led_max_Vout * this->setpoint_low_current_percent;
        this->setpoint_high_voltage = this->led_max_Vout * this->setpoint_high_current_percent;
    }

    ESP_LOGD(TAG, "Calibration Info: %f%%: %fV, %f%%: %fV", 
        this->setpoint_low_current_percent,
        this->setpoint_low_voltage,
        this->setpoint_high_current_percent,
        this->setpoint_high_voltage);
}

//Simple Binary Search for Voltage in Current Range 
//-> Search for Two Points for Calibration (Voltage to Current % (Current % ~ Brightness %))
//
// Does not work! - maybe for integration constraints on current measurements 
// Adding delays enables current measurements but voltage then is not set accordingly
float SiC45XComponent::searchVoltageAtCurrent(float currentMin, float currentMax){
    float left = 0;
    float right = this->led_max_Vout;
    while(left <= right){
        float mid = left + (right - left) / 2;
        this->setVoltage(mid);
        //wait 500ms for adjustment
        float currentAtMid = ic.getReadIout();
        ESP_LOGD(TAG, "CALIB: left: %fV, mid: %fV, right: %fV, IatMid: %fA", left, mid, right, currentAtMid);
        if(currentAtMid >= (currentMin) && 
           currentAtMid <= (currentMax)){
            //current at output is between min and max
            return mid;
        }
        if(currentAtMid < currentMin){
            left = mid;
        }else{
            right = mid;
        }

        //left will never reach max vout
        if(left >= this->led_max_Vout * .95){
            ESP_LOGE(TAG, "CURRENT NEVER REACHED %fA", currentMin);
            this->setVoltage(0);
            return -1;
        }
        if(right <= this->led_max_Vout * 0.05){
            ESP_LOGE(TAG, "CURRENT ALWAYS EXCEEDS %fA", currentMax);
            this->setVoltage(0);
            return -1;
        }
    }
    this->setVoltage(0);
    return -1;
}

//0 <= Brightness <= 1.0
void SiC45XComponent::setBrightnessPercent(float brightness){
    if(brightness < 0 ||  brightness > 1.0){
        ESP_LOGE(TAG, "BRIGHTNESS VALUE INVALID");
        return;
    }
    if(this->setpoint_low_voltage == -1 || this->setpoint_high_voltage == -1){
        ESP_LOGE(TAG, "CANNOT SET BRIGHTNESS, CALIBRATION FAILED!");
    }
    // y = mx+b (use mean values for X in one percentile and 99th percentile search)
    
    // b
    float slope = (this->setpoint_high_voltage - this->setpoint_low_voltage) / 
                  (this->setpoint_high_current_percent - this->setpoint_low_current_percent);
    // y
    float voltage = this->setpoint_low_voltage + slope * 
                    (brightness - this->setpoint_low_current_percent);
    voltage = voltage * this->output_power_limit_percent; //output power limit (%)

    if(brightness == 0){
        voltage = this->led_max_Vout * this->output_power_limit_percent * 0.01;
    }
    if(voltage > this->led_max_Vout){
        voltage = this->led_max_Vout; //ensure Vout not higher then this->led_max_Vout
    }

    //apply to output
    ESP_LOGD(TAG, "Setting Voltage: %fV (Brightness %f)", voltage, brightness);
    setVoltage(voltage);
}

void SiC45XComponent::setVoltage(float voltage){
    //Set correct scale loop (if necessary)
    if(voltage <= 1.8){
        ic.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_0V3_1V8);
    }else if(voltage <= 3.3){
        ic.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_1V8_3V3);
    }else if(voltage <= 5.0){
        ic.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_3V3_5V0);
    }else if(voltage <= 12.0){
        ic.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_5V0_12V0);
    }
    //could use PGOOD to indicate that output voltage is within set bounds
    //to light led or digital pin -> ignoring for now
    //ic.setPowerGoodOn(0.1);
    //ic.setPowerGoodOff(0.05);

    //Set VoutCommand
    if(voltage <= 12.0 && voltage <= this->led_max_Vout){
        ic.setVoutCommand(voltage);
    }else{
        ESP_LOGE(TAG, "TRIED SETTING VOLTAGE %fV > 12V IGNORED", voltage);
    }
}

void SiC45XComponent::dump_config() {
}

float SiC45XComponent::get_setup_priority() const { return setup_priority::LATE; }

void SiC45XComponent::call_setup() {
    this->setup();
    this->start_poller();
}

void SiC45XComponent::start_poller() {
    this->set_interval("update", this->get_update_interval(), [this]() { this->update(); });
}

void SiC45XComponent::stop_poller() {
    this->cancel_interval("update");
}

}  // namespace sic45x_led
}  // namespace esphome