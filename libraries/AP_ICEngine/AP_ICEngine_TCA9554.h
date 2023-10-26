/*
  optional control of starter via a TCA9554 I2C
 */

#include "AP_ICEngine_config.h"

#if AP_ICENGINE_TCA9554_STARTER_ENABLED
#include "AP_ICEngine.h"


/*
 * TCA9554 output register mapping for PMB Rev E
 * P0 = PMU_EN - PMU output ON/OFF  (CN6 pin 2)
 * P1 = ECU_EN - Unused (previously Engine Kill Switch)
 * P2 = I2C_P2 - Unused
 * P3 = LED (active low)
 * P4 = PMU_START - Crank Direction (CN6 pin 5)
 * P5 = PMU_ARM  - Crank Signal (CN6 pin 6)
 * P6 = PMU_STAT_IN - Unused
 * P7 = PMU_STAT - Unused
 */

#define TCA9554_I2C_BUS      1
#define TCA9554_I2C_ADDR     0x20
#define TCA9554_OUTPUT       0x01  // Output Port register address. Outgoing logic levels
#define TCA9554_OUT_DEFAULT  0x30  // 0011 0000
#define TCA9554_CONF         0x03  // Configuration Port register address [0 = Output]
#define TCA9554_PINS         0xC2  // Set all used ports to outputs = 1100 0010

class AP_ICEngine_TCA9554 {
public:
    void set_starter(bool on);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_TCA9554;

    enum TCA9554_state_t {
        STARTER_OFF = 0x30,	// output register - 0011 0000
        STARTER_ON  = 0x11, // output register - 0001 0001 - Forward direction
    };
    TCA9554_state_t last_state;

    bool initialised;

    bool TCA9554_init();
    void TCA9554_set(TCA9554_state_t value);
};

#endif // AP_ICENGINE_TCA9554_STARTER_ENABLED
