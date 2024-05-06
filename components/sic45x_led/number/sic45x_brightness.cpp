#include "sic45x_brightness.h"

namespace esphome::sic45x_led::brightness
{
    void SiC45X_Brightness::control(float value){
        controller->setBrightnessPercent(value / 100.0);
        publish_state(value);
    }
    void SiC45X_Brightness::dump_config(){}
} // esphome::SiC45XComponent::brightness