#pragma once

#ifdef HAL_PERIPH_ENABLE_FSO_POWER_STACK

#include <AP_DAC/AP_DAC.h>

extern const AP_HAL::HAL &hal;

class FSOPowerStack {
public:
    friend class AP_Periph_FW;
    FSOPowerStack(void);

    static const struct AP_Param::GroupInfo var_info[];

    void init(void);
    void update(void);

private:

    enum class Option : uint32_t {
        DEBUG = 0,
        CAL = 1
    };

    bool option_is_set(Option option) {
        return (options & (1U << uint32_t(option))) != 0;
    }

    AP_Int32 options;
    AP_Float payload_1_voltage;
    AP_Float payload_2_voltage;
    AP_Float payload_HV_current_max;
    AP_Float payload_1_current_max;
    AP_Float payload_2_current_max;
    AP_Float bec_temperature_max;
    AP_Float fan_1_min_Hz;
    AP_Float fan_2_min_Hz;
    AP_Float fan_3_min_Hz;
    AP_Float fan_4_min_Hz;
    AP_Float cal_main_voltage;
    AP_Float cal_main_current;
    AP_Float cal_HV_current;
    AP_Float cal_HCB_current;
    AP_Float cal_LCB_current;
    AP_Float cal_payload_P1c1;
    AP_Float cal_payload_P1c2;
    AP_Float cal_payload_P2c1;
    AP_Float cal_payload_P2c2;
    

    uint32_t last_update_ms;

    class Fan {
    public:
        Fan(uint8_t _instance, uint8_t _pin, const AP_Float &_min_Hz) :
            instance{_instance},
            pin{_pin},
            min_Hz{_min_Hz}
            { }
        void init();
        void update();
        void handler(uint8_t pin,
                     bool pin_state,
                     uint32_t timestamp);
        float freq_hz;
    private:
        const AP_Float &min_Hz;
        uint8_t instance;
        uint8_t pin;
        uint32_t last_pulse_us;
        uint32_t dt_sum;
        uint32_t dt_count;
    } fans[4] {
        { 0, FSO_FAN_TACH1_PIN, fan_1_min_Hz },
        { 1, FSO_FAN_TACH2_PIN, fan_2_min_Hz },
        { 2, FSO_FAN_TACH3_PIN, fan_3_min_Hz },
        { 3, FSO_FAN_TACH4_PIN, fan_4_min_Hz },
    };

    uint32_t last_fan_ms;
    void update_fans();

    void update_BECs();

    // States used during turn on
    enum TurnOnState {
        Off,
        PreChargeStart,
        PreCharge,
        On,
        ShutDown
    };

    // States used during turn on
    enum CalibrateState {
        Begin,
        Payload_BEC_C1,
        Payload_BEC_C2,
        Payload_BEC_1_Shunt,
        Payload_BEC_2_Shunt,
        Internal_BEC_HC_Shunt,
        Internal_BEC_1_Shunt,
        Internal_BEC_2_Shunt,
        Payload_HV_Shunt,
        Main_V_Divider,
        Bat1_Amp_Offset,
        Bat2_Amp_Offset,
        Finished
    };
    enum CalibrateSubState {
        Setup,
        Start,
        Measure,
        Write
    };

    CalibrateState cal_state = CalibrateState::Begin;
    CalibrateSubState cal_sub_state = CalibrateSubState::Setup;
    uint32_t    cal_ms;
    uint32_t    cal_measurement_count;
    float       cal_measurement_1;
    float       cal_measurement_2;
    float       cal_measurement_3;

    void update_switches();
    void update_main_power();
    void update_payload_HV_power();
    void update_payload_BEC();
    void update_internal_BEC();
    void update_DAC();
    void calibrate();

    void set_main_on(){main_on = true;}
    void set_main_off(){main_on = false;}
    
    bool main_is_on(){return main_state == TurnOnState::On;}
    bool main_is_off(){return main_state == TurnOnState::Off;}

    void set_payload_BEC_1_on(){hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 1);}
    void set_payload_BEC_1_off(){hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 0);}

    void set_payload_BEC_2_on(){hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 1);}
    void set_payload_BEC_2_off(){hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 0);}

    void set_switch_1_on(){switch_1_on = true;}
    void set_switch_1_off(){switch_1_on = false;}
    
    bool switch_1_is_on(){return switch_1_on == true;}
    bool switch_1_is_off(){return switch_1_on == false;}

    void set_switch_2_on(){switch_2_on = true;}
    void set_switch_2_off(){switch_2_on = false;}
    
    bool switch_2_is_on(){return switch_2_on == true;}
    bool switch_2_is_off(){return switch_2_on == false;}

    void set_LED_1_on(){hal.gpio->write(FSO_LED_MAIN_PIN, 1);}
    void set_LED_1_off(){hal.gpio->write(FSO_LED_MAIN_PIN, 0);}

    void set_LED_2_on(){hal.gpio->write(FSO_LED_PAYLOAD_PIN, 1);}
    void set_LED_2_off(){hal.gpio->write(FSO_LED_PAYLOAD_PIN, 0);}

    void set_LED_debug_on(){hal.gpio->write(FSO_LED_DEBUG_PIN, 1);}
    void set_LED_debug_off(){hal.gpio->write(FSO_LED_DEBUG_PIN, 0);}

    void set_main_PC_on(){hal.gpio->write(FSO_MAIN_PC_PIN, 1);}
    void set_main_PC_off(){hal.gpio->write(FSO_MAIN_PC_PIN, 0);}

    void set_bat_1_SW_on(){hal.gpio->write(FSO_BAT_1_EN_PIN, 1);}
    void set_bat_1_SW_off(){hal.gpio->write(FSO_BAT_1_EN_PIN, 0);}

    void set_bat_2_SW_on(){hal.gpio->write(FSO_BAT_2_EN_PIN, 1);}
    void set_bat_2_SW_off(){hal.gpio->write(FSO_BAT_2_EN_PIN, 0);}

    uint32_t last_report_ms;
    void report();


    bool switch_1_on;
    bool switch_1_switch_released;
    uint32_t switch_1_press_time_ms;

    bool switch_2_on;
    bool switch_2_switch_released;
    uint32_t switch_2_press_time_ms;

    bool main_on;
    bool power_on;
    uint32_t start_main_precharge_ms;

    class BEC {
    public:
        BEC(const char *_name, const char *shortname, uint8_t _battery_index);
        const char *name;
        const char *shortname;
        const uint8_t battery_index;

        float temperature;
        LowPassFilterFloat current_filter;

        virtual float max_current() const = 0;
        virtual float max_temperature() const = 0;

        void update();

    private:
        void check_current();
        void check_temperature();

        virtual void update_state() = 0;
        virtual void handle_over_current();
    };

    class InternalBEC : public BEC {
    public:
        using BEC::BEC;
        // Maximum current of the low current internal BECs:
        float max_current() const override { return 1.0; }
        // Maximum temperature of the low current internal BECs:
        float max_temperature() const override { return 100.0; }

        void update_state() override {};
    };
    class InternalHCBEC : public InternalBEC {
    public:
        using InternalBEC::InternalBEC;
        // Maximum current of the high current internal BEC:
        float max_current() const override { return 10.0; }
        // Maximum temperature of the low current internal BECs:
        float max_temperature() const override { return nanf(""); }

    };

    class PayloadBEC : public BEC {
    public:
        PayloadBEC(const char * _name, const char *_shortname, uint8_t _battery_index, AP_Float &_max_current_parameter, uint8_t _enable_pin) :
            BEC(_name, _shortname, _battery_index),
            max_current_parameter{_max_current_parameter},
            enable_pin{_enable_pin}
            { }
        virtual float max_current() const override {
            // return the smaller of the user-configured current and
            // the compiled-in limit:
            return MIN(fixed_max_current(), max_current_parameter);
        }
        float max_temperature() const override { return PAYLOAD_BEC_TEMPERATURE_MAX; }

    protected:
        // Maximum current of the payload BECs (Amps):
        virtual float fixed_max_current() const { return PAYLOAD_BEC_CURRENT_MAX; }
        virtual void handle_over_current() override;

        // turn the BEC on or off instantly:
        void on() { hal.gpio->write(enable_pin, 1); }
        void off() { hal.gpio->write(enable_pin, 0); }

        virtual void update_state() override;

        enum class DesiredState {
            OFF = 0,
            ON = 1,
        };
        void set_desired_state(DesiredState _desired_state) {
            desired_state = _desired_state;
        }

        DesiredState desired_state = DesiredState::OFF;

    private:

        // States used during turn on
        enum class State {
            OFF,
            ON,
        };
        State current_state = State::OFF;
        void set_state(State _state) {
            current_state = _state;
        }

        const AP_Float &max_current_parameter;
        const uint8_t enable_pin;
    };

    class PayloadHVBEC : public PayloadBEC{
    public:
        PayloadHVBEC(const char * _name, const char *_shortname, uint8_t _battery_index, AP_Float &_max_current_parameter, uint8_t _enable_pin, uint8_t _precharge_enable_pin) :
            PayloadBEC(_name, _shortname, _battery_index, _max_current_parameter, _enable_pin),
            precharge_enable_pin{_precharge_enable_pin}
            { }

        // States used during turn on
        enum class State {
            OFF,
            PRECHARGE,
            ON,
        };

    protected:
        // Maximum current of the HC payload output (Amps):
        float fixed_max_current() const override { return PAYLOAD_HV_CURRENT_MAX; }
        void update_state() override;
        virtual void handle_over_current() override;

    private:
        State current_state = State::OFF;
        void set_state(State _state) {
            current_state = _state;
        }

        float precharge_start_ms;
        uint8_t precharge_enable_pin;

        void precharge_on() {
            hal.gpio->write(precharge_enable_pin, 1);
        }
        void precharge_off() {
            hal.gpio->write(precharge_enable_pin, 0);
        }
    };

    static constexpr uint8_t FSO_MAX_BECS { 10 };
    BEC *becs[FSO_MAX_BECS];
    uint8_t num_becs;

    TurnOnState main_state = Off;

    AP_DAC dac;

    // init called after CAN init
    void late_init(void);

    bool done_late_init;

    // Maximum current of the HV payload output
    static constexpr float PAYLOAD_HV_CURRENT_MAX { 25.0 };
    // Maximum current of the payload BECs:
    static constexpr float PAYLOAD_BEC_CURRENT_MAX { 10.0 };
    // Maximum temperature of the payload BECs (degC):
    static constexpr float PAYLOAD_BEC_TEMPERATURE_MAX { 100.0 };
};

#endif // HAL_PERIPH_ENABLE_FSO_POWER_STACK


