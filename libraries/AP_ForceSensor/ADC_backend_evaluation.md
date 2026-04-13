# AP_ForceSensor: ADC Layering and New Backend Evaluation

## Devices under consideration

| Device | Bits | SPS (max) | PGA | Channels | Init complexity | I2C addresses |
|--------|------|-----------|-----|----------|-----------------|---------------|
| NAU7802 | 24 | 320 | 1–128× | 1 diff | High (LDO, AFE cal, bridge excitation) | 0x2A (fixed) |
| ADS1115 | 16 | 860 | 2/3–16× | 4 SE or 2 diff | Low (1 config register) | 0x48–0x4B |
| MCP3421 | 18 | 240 (12-bit), 3.75 (18-bit) | 1–8× | 1 diff | Very low (1 config byte) | 0x68–0x6F |

---

## Evaluation A: Extract AP_ADC_NAU7802 layer

**Conclusion: not recommended.**

The NAU7802's init embeds force-sensor-specific steps at its core: internal LDO enable
(for bridge excitation), 330 pF decoupling cap (strain-gauge application note §9.14),
and AFE calibration. These cannot be cleanly separated from a "generic ADC" init
without adding flags that leak the force-sensor concern into the ADC layer. The ADS1115
and MCP3421 have no such steps — their inits are a single register write — so a shared
`AP_ADC` base class would need to accommodate wildly different complexity levels for no
concrete gain.

There is also no second consumer of a hypothetical `AP_ADC_NAU7802` today. Abstractions
should be extracted when a concrete second use case exists, not speculatively.

---

## Evaluation B: Full AP_ADC intermediate layer (wrapping all three chips)

A proposed stack: `AP_ADC_Backend` → `AP_ADC_NAU7802` / `AP_ADC_ADS1115` /
`AP_ADC_MCP3421`, with a `AP_ForceSensor_ADC` backend consuming any `AP_ADC_Backend`.

**Not recommended.** The shared logic across ADS1115 and MCP3421 timer callbacks is
~30 lines (DRDY check → read raw → Newton convert). That does not justify a new
abstraction tier. The chips differ enough in config encoding, data width, and DRDY
mechanism that a shared base would be mostly virtual dispatch with little substance.
ArduPilot's existing pattern (AP_Baro, AP_RangeFinder) does not insert a chip-driver
tier between the framework backend and the hardware device.

The existing `AP_ADC_ADS1115` in `libraries/AP_ADC/` is Linux-HAL-specific (hardcoded
bus/address 0x48, mV output, OwnPtr ownership) and is not suitable for reuse here.

---

## Evaluation C: AP_ForceSensor_ADS1115 (direct backend)

**Recommended.** Direct I2C backend following the `AP_ForceSensor_NAU7802` pattern.

**Init:** write one 16-bit Config register — MUX (input select), PGA, MODE=continuous,
DR (data rate). No calibration required.

**Timer callback:** read 2-byte big-endian signed result from Conversion register
(0x00); check OS bit for data readiness.

**Backend-specific AP_Params** (exposed via AP_SUBGROUPVARPTR):

| Param | Description | Default |
|-------|-------------|---------|
| `MUX` | Input: 0=AIN0-AIN1 diff, 1=AIN0-AIN3, 2=AIN1-AIN3, 3=AIN2-AIN3, 4=AIN0 SE, 5=AIN1 SE, 6=AIN2 SE, 7=AIN3 SE | 0 |
| `GAIN` | PGA: 0=±6.144 V, 1=±4.096 V, 2=±2.048 V, 3=±1.024 V, 4=±0.512 V, 5=±0.256 V | 2 |
| `RATE` | DR: 0=8, 1=16, 2=32, 3=64, 4=128, 5=250, 6=475, 7=860 SPS | 4 |
| `ZERO` | Zero offset in raw counts (set by tare) | 0 |
| `SCALE` | Counts per Newton (0 = report raw counts) | 0 |

Common frontend params `FSCL1_ADDR` (0x48–0x4B) and `FSCL1_BUS` are reused as-is.

---

## Evaluation D: AP_ForceSensor_MCP3421 (direct backend)

**Recommended.** Simpler than ADS1115 — single 8-bit config byte, single differential
channel, no mux.

**Init:** write one byte `[RDY=1 | O/C=1 | S1 | S0 | G1 | G0]` for continuous
conversion at chosen resolution and gain.

**Timer callback:** I2C read returns 3 bytes (18-bit mode) or 2 bytes (12/14/16-bit)
plus a config byte. RDY flag (bit 7 of the returned config byte: 0 = fresh data) is
checked before consuming the result. Result is sign-extended from the configured width
to int32_t.

Data widths by resolution (S1:S0):
- 12-bit (00): 2 data bytes, result in bits 11–0
- 14-bit (01): 2 data bytes, result in bits 13–0
- 16-bit (10): 2 data bytes, result in bits 15–0
- 18-bit (11): 3 data bytes, result in bits 17–0

**Backend-specific AP_Params:**

| Param | Description | Default |
|-------|-------------|---------|
| `BITS` | 0=12-bit/240 SPS, 1=14-bit/60 SPS, 2=16-bit/15 SPS, 3=18-bit/3.75 SPS | 3 |
| `GAIN` | PGA: 0=1×, 1=2×, 2=4×, 3=8× | 0 |
| `ZERO` | Zero offset in raw counts (set by tare) | 0 |
| `SCALE` | Counts per Newton (0 = report raw counts) | 0 |

Common frontend params `FSCL1_ADDR` (0x68–0x6F) and `FSCL1_BUS` are reused as-is.

---

## Files to create/modify

| File | Action |
|------|--------|
| `libraries/AP_ForceSensor/AP_ForceSensor_config.h` | Add `AP_FORCESENSOR_ADS1115_ENABLED`, `AP_FORCESENSOR_MCP3421_ENABLED` |
| `libraries/AP_ForceSensor/AP_ForceSensor_ADS1115.h/.cpp` | New backend |
| `libraries/AP_ForceSensor/AP_ForceSensor_MCP3421.h/.cpp` | New backend |
| `libraries/AP_ForceSensor/AP_ForceSensor.h` | Add `Type::ADS1115 = 3`, `Type::MCP3421 = 4` |
| `libraries/AP_ForceSensor/AP_ForceSensor.cpp` | Add detect cases |
| `libraries/SITL/SIM_ADS1115.h/.cpp` | New SITL register simulator |
| `libraries/SITL/SIM_MCP3421.h/.cpp` | New SITL register simulator |
| `libraries/SITL/SIM_config.h` | Add `AP_SIM_ADS1115_ENABLED`, `AP_SIM_MCP3421_ENABLED` |
| `libraries/SITL/SIM_I2C.cpp` | Register ADS1115 at bus=0/0x48, MCP3421 at bus=0/0x68 |

---

## SITL simulators

Both follow the `SIM_NAU7802` pattern (`I2CRegisters_8Bit`):

**SIM_ADS1115:**
- Reg 0x00 (RDONLY): Conversion register — updated each simulated sample tick
- Reg 0x01 (RDWR): Config register — simulator reads MUX/PGA/DR bits to set interval
- Reg 0x02, 0x03 (RDWR): Lo/Hi threshold (must exist to avoid write panics)
- ADC value from `AP::sitl()->fscl_load` × scale factor

**SIM_MCP3421:**
- MCP3421 uses a non-standard I2C protocol (variable-length read, single config byte
  write). `I2CRegisters_8Bit` is a poor fit; implement a custom `rdwr()` that:
  - On write (1 msg): stores the config byte
  - On read (1 msg, I2C_M_RD): returns data bytes + config byte, width from BITS setting
- ADC value from `AP::sitl()->fscl_load` × scale factor, sign-extended to configured width
