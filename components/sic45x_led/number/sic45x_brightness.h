#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "../sic45x_led.h"

namespace esphome::sic45x_led::brightness
{
    static const char *TAG = "sic45x_led_brightness";
    class SiC45X_Brightness : public number::Number, public Component
    {
    public:
        void setup(){ publish_state(0); }
        void control(float value);
        void dump_config() override;
        void set_controller(SiC45XComponent *controller){ this->controller = controller; }

    protected:
        SiC45XComponent *controller;
    };

    static void set_reference(SiC45X_Brightness *brightness, SiC45XComponent *controller){
        brightness->set_controller(controller);
    }

    static void publish_state(SiC45X_Brightness *brightness, int value){
        brightness->publish_state(value);
    }
} // namespace esphome::SiC45XComponent::brightness