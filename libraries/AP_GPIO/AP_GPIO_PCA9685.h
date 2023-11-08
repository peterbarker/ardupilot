#include "AP_GPIO_config.h"

#if AP_GPIO_PCA9685_ENABLED

#include "AP_GPIO_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_GPIO_PCA9685 : public AP_GPIO_Backend {
public:

    using AP_GPIO_Backend::AP_GPIO_Backend;

    CLASS_NO_COPY(AP_GPIO_PCA9685);

    static AP_GPIO_Backend *probe(AP_GPIO_Backend_params &params);

private:

    bool init();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

};

#endif  // AP_GPIO_PCA9685_ENABLED
