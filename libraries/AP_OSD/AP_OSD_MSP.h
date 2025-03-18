#include <AP_OSD/AP_OSD_Backend.h>
#include <AP_MSP/AP_MSP.h>

class AP_OSD_MSP : public AP_OSD_Backend
{
    using AP_OSD_Backend::AP_OSD_Backend;
public:
    static AP_OSD_Backend *probe(AP_OSD &osd);

    //initilize display port and underlying hardware
    bool init() override;

    //draw given text to framebuffer
    void write(uint8_t x, uint8_t y, const char* text) override {};

    //flush framebuffer to screen
    void flush() override {};

    //clear framebuffer
    void clear() override {};

    bool is_compatible_with_backend_type(AP_OSD::Type type) const override {
        switch(type) {
        case AP_OSD::Type::MSP:
        case AP_OSD::Type::MSP_DISPLAYPORT:
            return false;
        case AP_OSD::Type::NONE:
        case AP_OSD::Type::TXONLY:
        case AP_OSD::Type::MAX7456:
        case AP_OSD::Type::SITL:
            return true;
        }
        return false;
    }

    AP_OSD::Type get_backend_type() const override {
        return AP_OSD::Type::MSP;
    }
private:
    void setup_defaults(void);
};
