#include <Arduino.h>
#include <Wire.h>
#include <driver/pcnt.h>
#include <esp_err.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

/*
  FURUTA PENDULUM -- MATLAB BENCH COMMUNICATION FIRMWARE v1
  Target: ESP32-S3, PlatformIO Arduino framework.

  Exact user wiring:
  ACS712 OUT -> GPIO4 (DIRECT, hardware overvoltage risk; see below)
  Motor encoder A/B -> GPIO5/GPIO6
  AS5600 SDA/SCL -> GPIO8/GPIO9, 3.3V, I2C 0x36
  TB6612 PWMA/AIN1/AIN2/STBY -> GPIO15/16/17/18
  TB6612 STBY has external 10k pull-down.

  IMPORTANT ELECTRICAL RISKS:
  - A 5V-powered ACS712-5A can produce >3.3V at GPIO4 during
    positive current transients. SOFTWARE LIMITS DO NOT PROTECT THE PIN.
    Add a suitable analog interface or disconnect OUT before powered tests.
  - TB6612 continuous motor current capability is limited. An
    adjustable current-limited motor supply, physical E-stop and
    verified motor stall current are required for powered testing.
  - STOP disables the bridge (motor coasts); it is NOT active braking.

  Does not implement Furuta stabilization or energy-based swing-up.
  All motion commands have finite duration; no infinite jog command.
  AS5600 magnet weak (ML=1) is reported but does not disable
  OPEN-LOOP experiments. Do NOT use invalid angle for closed-loop control.

  Protocol: ASCII lines ending '\n' at 115200 baud.
  Commands: HELLO, STATUS, HELP, STOP, ESTOP, RESET_FAULT,
    ZERO_ARM, ZERO_PEND_DOWN, CAL_CURRENT,
    STREAM,<0..100>, SET,<name>,<value>,
    PULSE,<signed_percent>,<duration_ms>,
    WAVE,<SIN|TRI|SQUARE|CHIRP|PRBS>,<amp_pct>,<f0_hz>,
         <f1_hz>,<duration_ms>,<offset_pct>

  Telemetry fields (CSV):
    DATA,t_ms,armCount,armDeg,armVelDegS,pendRawDeg,pendDeg,
         pendVelDegS,currentMv,currentA,pwmPct,mode,magFlags,faultFlags
  Values 'nan' indicate uncalibrated or unavailable measurements.
*/

namespace Pins {
constexpr uint8_t current = 4;
constexpr uint8_t encoderA = 5;
constexpr uint8_t encoderB = 6;
constexpr uint8_t sda = 8;
constexpr uint8_t scl = 9;
constexpr uint8_t pwm = 15;
constexpr uint8_t in1 = 16;
constexpr uint8_t in2 = 17;
constexpr uint8_t standby = 18;
}

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_CLOCK_HZ = 100000; // proven reliable in earlier testing
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
constexpr int PCNT_LIMIT = 30000;
constexpr uint32_t SENSOR_PERIOD_MS = 5; // 200 Hz encoder/pendulum acquisition
constexpr uint32_t ADC_PERIOD_MS = 2;    // 500 Hz current acquisition
constexpr uint32_t AS_DIAGNOSTIC_PERIOD_MS = 250;
constexpr uint32_t AS_RETRY_MS = 2000;
constexpr uint32_t MOTOR_UPDATE_MS = 5;
constexpr uint32_t PWM_HZ = 20000;
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_FULL_SCALE = (1UL << PWM_BITS) - 1;
constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint32_t MAX_PULSE_MS = 3000;
constexpr uint32_t MAX_WAVE_MS = 10000;
constexpr uint32_t MIN_RESTART_MS = 300;
constexpr float MAX_ALLOWED_DUTY_PERCENT = 60.0f;
constexpr float CURRENT_MV_PER_AMP = 185.0f;  // ACS712-5A, direct ADC
constexpr uint32_t CURRENT_ZERO_TIME_MS = 1000;

// Fault flag bits exported in telemetry.
constexpr uint8_t FAULT_ESTOP = 1u << 0;
constexpr uint8_t FAULT_OVERCURRENT = 1u << 1;
constexpr uint8_t FAULT_ADC_WARNING = 1u << 2;
constexpr uint8_t FAULT_ARM_LIMIT = 1u << 3;
constexpr uint8_t FAULT_ENCODER = 1u << 4;
constexpr uint8_t FAULT_I2C_LOST = 1u << 5;

struct Config {
    double encoderCpr = 44.0;   // 11 PPR/channel, x4. CHECK motor datasheet.
    double gearboxRatio = 1.0; // encoder shaft revolutions per arm revolution
    double maxDuty = 35.0;     // software magnitude limit, NOT current limiting
    double armLimitDeg = 120.0;// relative to ZERO_ARM (coarse if 44CPR)
    double currentTripA = 0.9;// experimental cutoff, NOT electrical protection
    uint32_t telemetryHz = 25;
} cfg;

struct AS5600State {
    bool present = false;
    bool readable = false;
    bool zeroSet = false;
    bool tracking = false;
    uint16_t raw = 0;
    uint16_t downZero = 0;
    uint16_t prevRaw = 0;
    uint8_t status = 0;
    uint8_t agc = 0;
    uint16_t magnitude = 0;
    int32_t travelCounts = 0;
    float rawDeg = NAN;
    float pendDeg = NAN;    // 0 upright, 180 down
    float velocityDegS = 0;
    uint32_t lastValidMs = 0;
    uint32_t nextTryMs = 0;
    uint32_t errorCount = 0;
} as;

struct MotorEncoder {
    bool ready = false;
    bool zeroSet = false;
    int16_t previousHw = 0;
    int64_t count = 0;
    int64_t zeroCount = 0;
    int64_t lastVelocityCount = 0;
    float angleDeg = NAN;
    float velocityDegS = 0;
    uint32_t lastVelocityMs = 0;
} enc;

struct CurrentSensor {
    uint32_t lastMv = 0;
    float filteredMv = NAN;
    float zeroMv = NAN;
    float currentA = NAN;
    bool calibrated = false;
    bool calibrating = false;
    uint32_t calibrationStart = 0;
    uint32_t samples = 0;
    double sum = 0;
    double sumSq = 0;
    uint32_t overCurrentSince = 0;
} currentSensor;

enum class Motion : uint8_t { IDLE, PULSE, SIN, TRI, SQUARE, CHIRP, PRBS };
struct MotorExperiment {
    Motion mode = Motion::IDLE;
    float amplitude = 0;
    float offset = 0;
    float f0 = 0;
    float f1 = 0;
    uint32_t durationMs = 0;
    uint32_t startedMs = 0;
    uint32_t lastStopMs = 0;
    float appliedDuty = 0;
} experiment;

char inputLine[150];
size_t inputLength = 0;
uint32_t lastSensorMs = 0;
uint32_t lastAdcMs = 0;
uint32_t lastMagMs = 0;
uint32_t lastMotorMs = 0;
uint32_t lastTelemMs = 0;
uint8_t faults = 0;
bool pwmReady = false;
int lastMotorDirection = 0;

const char* motionName(Motion mode) {
    switch (mode) {
        case Motion::PULSE: return "PULSE";
        case Motion::SIN: return "SIN";
        case Motion::TRI: return "TRI";
        case Motion::SQUARE: return "SQUARE";
        case Motion::CHIRP: return "CHIRP";
        case Motion::PRBS: return "PRBS";
        default: return "IDLE";
    }
}

void setPwmRaw(uint32_t value) {
    if (!pwmReady) return;
    if (value > PWM_FULL_SCALE) value = PWM_FULL_SCALE;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(Pins::pwm, value);
#else
    ledcWrite(PWM_CHANNEL, value);
#endif
}

// Electrical standby means coast / high impedance, not instant braking.
void motorOff() {
    digitalWrite(Pins::standby, LOW);
    setPwmRaw(0);
    digitalWrite(Pins::in1, LOW);
    digitalWrite(Pins::in2, LOW);
    experiment.appliedDuty = 0.0f;
    lastMotorDirection = 0;
}

void stopExperiment(const char* why, bool announce = true) {
    const bool running = experiment.mode != Motion::IDLE;
    motorOff();
    experiment.mode = Motion::IDLE;
    experiment.lastStopMs = millis();
    if (running && announce) {
        Serial.printf("EVENT,STOP,%s\n", why);
    }
}

void latchFault(uint8_t bit, const char* reason) {
    stopExperiment("FAULT", false);
    if ((faults & bit) == 0) {
        faults |= bit;
        Serial.printf("EVENT,FAULT,%u,%s\n", (unsigned)faults, reason);
    }
}

bool initializePWM() {
    pinMode(Pins::standby, OUTPUT);
    pinMode(Pins::in1, OUTPUT);
    pinMode(Pins::in2, OUTPUT);
    digitalWrite(Pins::standby, LOW);
    digitalWrite(Pins::in1, LOW);
    digitalWrite(Pins::in2, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    pwmReady = ledcAttach(Pins::pwm, PWM_HZ, PWM_BITS);
#else
    pwmReady = ledcSetup(PWM_CHANNEL, PWM_HZ, PWM_BITS) != 0;
    if (pwmReady) ledcAttachPin(Pins::pwm, PWM_CHANNEL);
#endif
    motorOff();
    return pwmReady;
}

void applyDuty(float percent) {
    percent = constrain(percent, -float(cfg.maxDuty), float(cfg.maxDuty));
    if (fabsf(percent) < 0.2f) {
        motorOff();
        return;
    }
    const int direction = percent > 0 ? 1 : -1;
    // Only change direction and toggle STBY when the sign changes.
    // Do not repeatedly interrupt the bridge every 5 ms.
    if (direction != lastMotorDirection) {
        digitalWrite(Pins::standby, LOW);
        setPwmRaw(0);
        digitalWrite(Pins::in1, direction > 0 ? HIGH : LOW);
        digitalWrite(Pins::in2, direction > 0 ? LOW : HIGH);
        lastMotorDirection = direction;
    }
    setPwmRaw((uint32_t)lroundf(fabsf(percent) * PWM_FULL_SCALE / 100.0f));
    digitalWrite(Pins::standby, HIGH);
    experiment.appliedDuty = percent;
}

void initializeEncoder() {
    pinMode(Pins::encoderA, INPUT_PULLUP);
    pinMode(Pins::encoderB, INPUT_PULLUP);
    pcnt_config_t p = {};
    p.unit = PCNT_UNIT;
    p.counter_h_lim = PCNT_LIMIT;
    p.counter_l_lim = -PCNT_LIMIT;
    p.channel = PCNT_CHANNEL_0;
    p.pulse_gpio_num = Pins::encoderA;
    p.ctrl_gpio_num = Pins::encoderB;
    p.lctrl_mode = PCNT_MODE_KEEP;
    p.hctrl_mode = PCNT_MODE_REVERSE;
    p.pos_mode = PCNT_COUNT_INC;
    p.neg_mode = PCNT_COUNT_DEC;
    esp_err_t first = pcnt_unit_config(&p);
    p.channel = PCNT_CHANNEL_1;
    p.pulse_gpio_num = Pins::encoderB;
    p.ctrl_gpio_num = Pins::encoderA;
    p.pos_mode = PCNT_COUNT_DEC;
    p.neg_mode = PCNT_COUNT_INC;
    esp_err_t second = pcnt_unit_config(&p);
    if (first != ESP_OK || second != ESP_OK ||
        pcnt_counter_pause(PCNT_UNIT) != ESP_OK ||
        pcnt_set_filter_value(PCNT_UNIT, 100) != ESP_OK ||
        pcnt_filter_enable(PCNT_UNIT) != ESP_OK ||
        pcnt_counter_clear(PCNT_UNIT) != ESP_OK ||
        pcnt_counter_resume(PCNT_UNIT) != ESP_OK) {
        latchFault(FAULT_ENCODER, "PCNT_INIT");
        return;
    }
    enc.ready = true;
    enc.lastVelocityMs = millis();
}

void readMotorEncoder() {
    if (!enc.ready) return;
    int16_t hw = 0;
    if (pcnt_get_counter_value(PCNT_UNIT, &hw) != ESP_OK) {
        enc.ready = false;
        latchFault(FAULT_ENCODER, "PCNT_READ");
        return;
    }
    // PCNT hardware resets to zero at +/-limit. Samples must remain
    // far less than 15000 decoded edges apart for this wrap correction.
    int delta = (int)hw - (int)enc.previousHw;
    if (delta > PCNT_LIMIT / 2) delta -= PCNT_LIMIT;
    if (delta < -PCNT_LIMIT / 2) delta += PCNT_LIMIT;
    enc.count += delta;
    enc.previousHw = hw;
    const double countsPerArmRev = cfg.encoderCpr * cfg.gearboxRatio;
    if (enc.zeroSet) {
        enc.angleDeg = (float)((enc.count - enc.zeroCount) * 360.0 / countsPerArmRev);
    }
    uint32_t now = millis();
    // Longer differentiation window is essential for a 44-count motor encoder.
    if (now - enc.lastVelocityMs >= 40) {
        const float dt = (now - enc.lastVelocityMs) * 0.001f;
        const float velocity = (float)((enc.count - enc.lastVelocityCount) *
                            360.0 / countsPerArmRev / dt);
        enc.velocityDegS = 0.6f * enc.velocityDegS + 0.4f * velocity;
        enc.lastVelocityCount = enc.count;
        enc.lastVelocityMs = now;
    }
}

bool readAS5600(uint8_t startRegister, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(startRegister);
    if (Wire.endTransmission(false) != 0) return false;
    const size_t n = Wire.requestFrom(AS5600_ADDRESS, length, true);
    if (n != length) {
        while (Wire.available()) Wire.read();
        return false;
    }
    for (uint8_t i = 0; i < length; ++i) buffer[i] = Wire.read();
    return true;
}

bool probeAS5600() {
    Wire.beginTransmission(AS5600_ADDRESS);
    return Wire.endTransmission(true) == 0;
}

int16_t wrappedEncoderDelta(uint16_t newer, uint16_t older) {
    int16_t delta = (int16_t)newer - (int16_t)older;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    return delta;
}

float wrapAngle180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void sampleAS5600() {
    const uint32_t now = millis();
    if (!as.present) {
        as.readable = false;
        if (now < as.nextTryMs) return;
        as.nextTryMs = now + AS_RETRY_MS;
        as.present = probeAS5600();
        if (as.present) Serial.println("EVENT,AS5600,RECONNECTED");
        return;
    }
    uint8_t angle[2] = {};
    if (!readAS5600(0x0C, angle, 2)) {
        ++as.errorCount;
        as.present = false;
        as.readable = false;
        as.tracking = false;
        as.nextTryMs = now + AS_RETRY_MS;
        Serial.println("EVENT,AS5600,READ_FAILED_RETRY_2S");
        if (experiment.mode != Motion::IDLE) latchFault(FAULT_I2C_LOST, "AS5600_LOST");
        return;
    }
    as.raw = ((angle[0] & 0x0F) << 8) | angle[1];
    as.rawDeg = as.raw * (360.0f / 4096.0f);
    if (as.tracking && now - as.lastValidMs <= 100) {
        int16_t d = wrappedEncoderDelta(as.raw, as.prevRaw);
        as.travelCounts += d;
        float v = d * (360.0f / 4096.0f) * 1000.0f /
                  float((now - as.lastValidMs) ? (now - as.lastValidMs) : 1U);
        as.velocityDegS = 0.75f * as.velocityDegS + 0.25f * v;
    } else {
        as.velocityDegS = 0.0f;
    }
    as.tracking = true;
    as.readable = true;
    as.prevRaw = as.raw;
    as.lastValidMs = now;
    if (as.zeroSet) {
        // Down = +180 deg, upright = 0 deg. 180 and -180 equivalent.
        float theta = 180.0f + wrappedEncoderDelta(as.raw, as.downZero) *
                                    (360.0f / 4096.0f);
        as.pendDeg = wrapAngle180(theta);
    }
    if (now - lastMagMs >= AS_DIAGNOSTIC_PERIOD_MS) {
        lastMagMs = now;
        uint8_t statusByte = 0;
        if (readAS5600(0x0B, &statusByte, 1)) as.status = statusByte;
        uint8_t gain = 0;
        if (readAS5600(0x1A, &gain, 1)) as.agc = gain;
        uint8_t mag[2] = {};
        if (readAS5600(0x1B, mag, 2)) {
            as.magnitude = ((mag[0] & 0x0F) << 8) | mag[1];
        }
    }
}

uint8_t magneticFlags() {
    uint8_t flags = 0;
    if (as.status & 0x20) flags |= 1; // magnet detected
    if (as.status & 0x10) flags |= 2; // too weak
    if (as.status & 0x08) flags |= 4; // too strong
    if (as.readable) flags |= 8;
    if (as.zeroSet) flags |= 16;
    return flags;
}

void sampleCurrent() {
    currentSensor.lastMv = analogReadMilliVolts(Pins::current);
    float mv = (float)currentSensor.lastMv;
    if (!isfinite(currentSensor.filteredMv)) currentSensor.filteredMv = mv;
    else currentSensor.filteredMv += 0.2f * (mv - currentSensor.filteredMv);
    if (currentSensor.calibrating) {
        currentSensor.samples++;
        currentSensor.sum += mv;
        currentSensor.sumSq += (double)mv * mv;
    }
    if (currentSensor.calibrated) {
        currentSensor.currentA =
            (currentSensor.filteredMv - currentSensor.zeroMv) / CURRENT_MV_PER_AMP;
    } else {
        currentSensor.currentA = NAN;
    }
    // WARNING: THESE SOFTWARE CHECKS DO NOT PREVENT ELECTRICAL OVERVOLTAGE.
    if (experiment.mode != Motion::IDLE) {
        if (currentSensor.lastMv >= 3000) {
            latchFault(FAULT_ADC_WARNING, "ADC_READING_HIGH");
            return;
        }
        if (currentSensor.calibrated &&
            fabsf(currentSensor.currentA) > cfg.currentTripA) {
            if (currentSensor.overCurrentSince == 0)
                currentSensor.overCurrentSince = millis();
            else if (millis() - currentSensor.overCurrentSince >= 15)
                latchFault(FAULT_OVERCURRENT, "CURRENT_TRIP");
        } else {
            currentSensor.overCurrentSince = 0;
        }
    } else {
        currentSensor.overCurrentSince = 0;
    }
}

void startCurrentCalibration() {
    stopExperiment("CALIBRATION", false);
    currentSensor.calibrated = false;
    currentSensor.currentA = NAN;
    currentSensor.calibrating = true;
    currentSensor.calibrationStart = millis();
    currentSensor.samples = 0;
    currentSensor.sum = 0;
    currentSensor.sumSq = 0;
    Serial.println("OK,CAL_CURRENT,STARTED_MOTOR_OFF");
}

void finishCurrentCalibration() {
    currentSensor.calibrating = false;
    if (currentSensor.samples < 100) {
        Serial.println("ERR,CAL_CURRENT,TOO_FEW_SAMPLES");
        return;
    }
    double mean = currentSensor.sum / currentSensor.samples;
    double variance = fmax(0.0, currentSensor.sumSq / currentSensor.samples - mean * mean);
    double sigma = sqrt(variance);
    if (mean < 2200 || mean > 2800 || sigma > 55.0) {
        Serial.printf("ERR,CAL_CURRENT,BAD_ZERO_mean=%.1f_sd=%.1f\n", mean, sigma);
        return;
    }
    currentSensor.zeroMv = (float)mean;
    currentSensor.calibrated = true;
    Serial.printf("OK,CAL_CURRENT,zero_mV=%.2f_sd_mV=%.2f_N=%lu\n",
                  mean, sigma, (unsigned long)currentSensor.samples);
}

void reportStatus() {
    Serial.printf("STATUS,firmware=1.0,stream_hz=%lu,i2c_hz=%lu,sensor_hz=200,adc_hz=500,pwm_hz=%lu,"
                  "encoder_cpr=%.3f,gear=%.3f,duty_limit=%.1f,arm_limit=%.1f,current_trip=%.2f,"
                  "arm_zero=%u,pend_zero=%u,current_zero=%u,as_valid=%u,mag_agc=%u,mag_magnitude=%u,faults=%u\n",
                  (unsigned long)cfg.telemetryHz, (unsigned long)I2C_CLOCK_HZ,
                  (unsigned long)PWM_HZ, cfg.encoderCpr, cfg.gearboxRatio,
                  cfg.maxDuty, cfg.armLimitDeg, cfg.currentTripA,
                  (unsigned)enc.zeroSet, (unsigned)as.zeroSet,
                  (unsigned)currentSensor.calibrated, (unsigned)as.readable,
                  (unsigned)as.agc, (unsigned)as.magnitude, (unsigned)faults);
}

void reportTelemetry() {
    if (cfg.telemetryHz == 0) return;
    const uint32_t now = millis();
    const uint32_t period = 1000 / cfg.telemetryHz;
    if (now - lastTelemMs < period) return;
    lastTelemMs = now;
    Serial.printf("DATA,%lu,%lld,%.4f,%.4f,%.4f,%.4f,%.4f,%lu,%.4f,%.3f,%s,%u,%u\n",
                 (unsigned long)now, (long long)enc.count,
                 enc.angleDeg, enc.velocityDegS,
                 as.rawDeg, as.pendDeg, as.velocityDegS,
                 (unsigned long)currentSensor.lastMv, currentSensor.currentA,
                 experiment.appliedDuty, motionName(experiment.mode),
                 (unsigned)magneticFlags(), (unsigned)faults);
}

bool isNumber(const char* text, double* out) {
    if (!text || !*text) return false;
    char* end = nullptr;
    double value = strtod(text, &end);
    if (end == text || *end != '\0' || !isfinite(value)) return false;
    *out = value;
    return true;
}

void errorReply(const char* cmd, const char* msg) {
    Serial.printf("ERR,%s,%s\n", cmd, msg);
}

bool mayStartMotion() {
    if (faults) { errorReply("MOTION", "LATCHED_FAULT_RESET_FIRST"); return false; }
    if (!pwmReady || !enc.ready) { errorReply("MOTION", "HARDWARE_NOT_READY"); return false; }
    if (!enc.zeroSet) { errorReply("MOTION", "ZERO_ARM_FIRST"); return false; }
    if (!currentSensor.calibrated) { errorReply("MOTION", "CAL_CURRENT_FIRST"); return false; }
    if (currentSensor.calibrating) { errorReply("MOTION", "CALIBRATION_ACTIVE"); return false; }
    if (experiment.mode != Motion::IDLE) { errorReply("MOTION", "STOP_PREVIOUS_TEST"); return false; }
    if (millis() - experiment.lastStopMs < MIN_RESTART_MS) {
        errorReply("MOTION", "WAIT_FOR_COOLDOWN"); return false;
    }
    if (fabsf(enc.angleDeg) >= cfg.armLimitDeg) {
        errorReply("MOTION", "ARM_OUTSIDE_SOFT_LIMIT"); return false;
    }
    if (!as.readable) { errorReply("MOTION", "AS5600_NOT_READING"); return false; }
    return true;
}

void updateMotor() {
    if (experiment.mode == Motion::IDLE) return;
    const uint32_t elapsed = millis() - experiment.startedMs;
    if (elapsed >= experiment.durationMs) {
        stopExperiment("DURATION_COMPLETE"); return;
    }
    if (!enc.ready || !enc.zeroSet || !isfinite(enc.angleDeg)) {
        latchFault(FAULT_ENCODER, "ENCODER_UNAVAILABLE"); return;
    }
    if (fabsf(enc.angleDeg) >= cfg.armLimitDeg) {
        latchFault(FAULT_ARM_LIMIT, "ARM_SOFT_LIMIT"); return;
    }
    float duty = 0;
    const float t = elapsed * 0.001f;
    constexpr float PI2 = 6.2831853071795864769f;
    const float frac = float(elapsed) / float(experiment.durationMs);
    if (experiment.mode == Motion::PULSE) duty = experiment.amplitude;
    else {
        const float f = experiment.f0 + (experiment.f1 - experiment.f0) * frac;
        const float phase = PI2 * (experiment.f0 * t +
                               0.5f * (experiment.f1 - experiment.f0) *
                               t * t / (experiment.durationMs * 0.001f));
        float wave = 0;
        switch (experiment.mode) {
            case Motion::SIN: wave = sinf(PI2 * experiment.f0 * t); break;
            case Motion::TRI: {
                float p = fmodf(experiment.f0 * t, 1.0f);
                wave = 4.0f * fabsf(p - 0.5f) - 1.0f;
                break;
            }
            case Motion::SQUARE: wave = sinf(PI2 * experiment.f0 * t) >= 0 ? 1.0f : -1.0f; break;
            case Motion::CHIRP: wave = sinf(phase); break;
            case Motion::PRBS: {
                // Deterministic pseudo-random +/-1 updated each half period.
                uint32_t k = (uint32_t)floorf(2.0f * f * t);
                uint32_t n = (k + 1U) * 1103515245UL + 12345UL;
                n ^= n >> 13; n *= 0x5bd1e995UL; n ^= n >> 15;
                wave = (n & 1U) ? 1.0f : -1.0f;
                break;
            }
            default: break;
        }
        duty = experiment.offset + experiment.amplitude * wave;
    }
    applyDuty(duty);
}

void beginPulse(double percent, double duration) {
    if (!mayStartMotion()) return;
    if (duration < 20 || duration > MAX_PULSE_MS ||
        fabs(percent) < 0.2 || fabs(percent) > cfg.maxDuty) {
        errorReply("PULSE", "BAD_DUTY_OR_DURATION"); return;
    }
    experiment.mode = Motion::PULSE;
    experiment.amplitude = (float)percent;
    experiment.offset = 0;
    experiment.startedMs = millis();
    experiment.durationMs = (uint32_t)duration;
    applyDuty(experiment.amplitude);
    Serial.printf("OK,PULSE,duty=%.2f_duration_ms=%lu\n",
                  percent, (unsigned long)experiment.durationMs);
}

void beginWave(const char* type, double amp, double f0, double f1,
               double duration, double offset) {
    Motion mode = Motion::IDLE;
    if (!strcmp(type, "SIN")) mode = Motion::SIN;
    if (!strcmp(type, "TRI")) mode = Motion::TRI;
    if (!strcmp(type, "SQUARE")) mode = Motion::SQUARE;
    if (!strcmp(type, "CHIRP")) mode = Motion::CHIRP;
    if (!strcmp(type, "PRBS")) mode = Motion::PRBS;
    if (mode == Motion::IDLE) { errorReply("WAVE", "UNKNOWN_WAVEFORM"); return; }
    if (amp < 0 || amp > cfg.maxDuty || fabs(offset) + amp > cfg.maxDuty ||
        f0 < 0.1 || f0 > 8.0 || f1 < 0.1 || f1 > 8.0 ||
        duration < 100 || duration > MAX_WAVE_MS) {
        errorReply("WAVE", "BAD_AMPLITUDE_FREQUENCY_OR_DURATION"); return;
    }
    if (!mayStartMotion()) return;
    experiment.mode = mode;
    experiment.amplitude = (float)amp;
    experiment.offset = (float)offset;
    experiment.f0 = (float)f0;
    experiment.f1 = (float)f1;
    experiment.durationMs = (uint32_t)duration;
    experiment.startedMs = millis();
    Serial.printf("OK,WAVE,%s_amp=%.1f_f0=%.2f_f1=%.2f_ms=%lu_offset=%.1f\n",
                  type, amp, f0, f1, (unsigned long)experiment.durationMs, offset);
}

void setParameter(const char* name, double value) {
    if (experiment.mode != Motion::IDLE) { errorReply("SET", "STOP_MOTOR_FIRST"); return; }
    if (!strcmp(name, "ENC_CPR") && value >= 1 && value <= 100000) {
        cfg.encoderCpr = value;
        enc.lastVelocityCount = enc.count;
        Serial.printf("OK,SET,ENC_CPR=%.3f\n", cfg.encoderCpr);
    } else if (!strcmp(name, "GEAR") && value >= 1 && value <= 500) {
        cfg.gearboxRatio = value;
        enc.lastVelocityCount = enc.count;
        Serial.printf("OK,SET,GEAR=%.3f\n", cfg.gearboxRatio);
    } else if (!strcmp(name, "DUTY_LIMIT") && value >= 1 && value <= MAX_ALLOWED_DUTY_PERCENT) {
        cfg.maxDuty = value;
        Serial.printf("OK,SET,DUTY_LIMIT=%.2f\n", cfg.maxDuty);
    } else if (!strcmp(name, "ARM_LIMIT") && value >= 10 && value <= 360) {
        cfg.armLimitDeg = value;
        Serial.printf("OK,SET,ARM_LIMIT=%.2f\n", cfg.armLimitDeg);
    } else if (!strcmp(name, "CURRENT_TRIP") && value >= 0.2 && value <= 2.0) {
        cfg.currentTripA = value;
        Serial.printf("OK,SET,CURRENT_TRIP=%.3f\n", cfg.currentTripA);
    } else {
        errorReply("SET", "UNKNOWN_PARAMETER_OR_OUT_OF_RANGE");
    }
}

void printHelp() {
    Serial.println("HELP,HELLO;STATUS;STOP;ESTOP;RESET_FAULT;ZERO_ARM;ZERO_PEND_DOWN;CAL_CURRENT;STREAM,hz;");
    Serial.println("HELP,SET,ENC_CPR|GEAR|DUTY_LIMIT|ARM_LIMIT|CURRENT_TRIP,value");
    Serial.println("HELP,PULSE,signedDutyPercent,durationMs (20..3000)");
    Serial.println("HELP,WAVE,SIN|TRI|SQUARE|CHIRP|PRBS,amplitudePercent,fStartHz,fEndHz,durationMs,offsetPercent");
}

void handleCommand(char* line) {
    // Uppercase ASCII command line, comma separated.
    for (char* c = line; *c; ++c) *c = (char)toupper((unsigned char)*c);
    char* fields[9] = {};
    size_t n = 0;
    char* save = nullptr;
    for (char* token = strtok_r(line, ",", &save);
         token && n < 9; token = strtok_r(nullptr, ",", &save)) {
        fields[n++] = token;
    }
    if (n == 0) return;
    const char* cmd = fields[0];
    if (!strcmp(cmd, "HELLO")) {
        Serial.println("HELLO,FURUTA_BENCH,1.0,115200");
        reportStatus();
    } else if (!strcmp(cmd, "HELP")) {
        printHelp();
    } else if (!strcmp(cmd, "STATUS")) {
        reportStatus();
    } else if (!strcmp(cmd, "STOP")) {
        stopExperiment("HOST_STOP");
        Serial.println("OK,STOP,DRIVER_DISABLED_COAST");
    } else if (!strcmp(cmd, "ESTOP")) {
        latchFault(FAULT_ESTOP, "HOST_ESTOP");
        Serial.println("OK,ESTOP,LATCHED");
    } else if (!strcmp(cmd, "RESET_FAULT")) {
        stopExperiment("RESET", false);
        if (currentSensor.calibrating) { errorReply(cmd, "CALIBRATION_IN_PROGRESS"); return; }
        if (!enc.ready) { errorReply(cmd, "ENCODER_FAULT"); return; }
        faults = 0;
        Serial.println("OK,RESET_FAULT,CLEARED_CHECK_PHYSICAL_SAFETY");
    } else if (!strcmp(cmd, "ZERO_ARM")) {
        if (experiment.mode != Motion::IDLE) { errorReply(cmd, "STOP_MOTOR_FIRST"); return; }
        if (!enc.ready) { errorReply(cmd, "ENCODER_NOT_READY"); return; }
        readMotorEncoder();
        enc.zeroCount = enc.count;
        enc.angleDeg = 0;
        enc.zeroSet = true;
        Serial.println("OK,ZERO_ARM,CURRENT_ARM_POSITION_IS_ZERO");
    } else if (!strcmp(cmd, "ZERO_PEND_DOWN")) {
        if (!as.readable) { errorReply(cmd, "AS5600_NOT_READABLE"); return; }
        as.downZero = as.raw;
        as.pendDeg = 180.0f;
        as.zeroSet = true;
        Serial.printf("OK,ZERO_PEND_DOWN,raw=%u_down=180_up=0_magflags=%u\n",
                      (unsigned)as.raw, (unsigned)magneticFlags());
    } else if (!strcmp(cmd, "CAL_CURRENT")) {
        if (experiment.mode != Motion::IDLE) { errorReply(cmd, "MOTOR_RUNNING"); return; }
        startCurrentCalibration();
    } else if (!strcmp(cmd, "STREAM") && n == 2) {
        double hz;
        if (!isNumber(fields[1], &hz) || hz < 0 || hz > 100 ||
            fabs(hz - floor(hz)) > 0.001) {
            errorReply(cmd, "RANGE_0_TO_100_INTEGER"); return;
        }
        cfg.telemetryHz = (uint32_t)hz;
        Serial.printf("OK,STREAM,%lu_HZ\n", (unsigned long)cfg.telemetryHz);
    } else if (!strcmp(cmd, "SET") && n == 3) {
        double value;
        if (!isNumber(fields[2], &value)) { errorReply(cmd, "NOT_A_NUMBER"); return; }
        setParameter(fields[1], value);
    } else if (!strcmp(cmd, "PULSE") && n == 3) {
        double pwm, duration;
        if (!isNumber(fields[1], &pwm) || !isNumber(fields[2], &duration)) {
            errorReply(cmd, "INVALID_NUMBER"); return;
        }
        beginPulse(pwm, duration);
    } else if (!strcmp(cmd, "WAVE") && n == 7) {
        double amp, f0, f1, ms, offset;
        if (!isNumber(fields[2], &amp) || !isNumber(fields[3], &f0) ||
            !isNumber(fields[4], &f1) || !isNumber(fields[5], &ms) ||
            !isNumber(fields[6], &offset)) {
            errorReply(cmd, "INVALID_NUMBER"); return;
        }
        beginWave(fields[1], amp, f0, f1, ms, offset);
    } else {
        errorReply(cmd, "UNKNOWN_OR_WRONG_ARGUMENT_COUNT");
    }
}

void pollSerial() {
    // Bounded parser; keeps sensor and motor services responsive.
    for (int i = 0; i < 96 && Serial.available(); ++i) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (inputLength > 0) {
                inputLine[inputLength] = 0;
                handleCommand(inputLine);
                inputLength = 0;
            }
        } else if (c >= 32 && c <= 126) {
            if (inputLength < sizeof(inputLine) - 1) inputLine[inputLength++] = c;
            else {
                inputLength = 0;
                errorReply("PARSER", "LINE_TOO_LONG");
            }
        }
    }
}

void setup() {
    // STBY external 10k pulldown enforces off during boot.
    digitalWrite(Pins::standby, LOW);
    pinMode(Pins::standby, OUTPUT);
    digitalWrite(Pins::standby, LOW);
    Serial.begin(SERIAL_BAUD);
    delay(350);
    if (!initializePWM()) latchFault(FAULT_ESTOP, "PWM_INIT_FAIL");
    analogReadResolution(12);
    analogSetPinAttenuation(Pins::current, ADC_11db);
    initializeEncoder();
    Wire.begin(Pins::sda, Pins::scl);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(20);
    as.present = probeAS5600();
    as.nextTryMs = millis() + AS_RETRY_MS;
    lastSensorMs = lastAdcMs = lastMagMs = lastTelemMs = millis();
    experiment.lastStopMs = millis();
    Serial.println("HELLO,FURUTA_BENCH,1.0,115200");
    Serial.println("EVENT,SAFETY,12V_ONLY_WITH_CURRENT_LIMIT_AND_SAFE_ADC_INTERFACE");
    Serial.println("EVENT,SAFETY,ACS712_DIRECT_GPIO4_IS_OVERVOLTAGE_RISK");
    Serial.println("EVENT,SAFETY,STOP_IS_COAST_NOT_BRAKE");
    reportStatus();
}

void loop() {
    const uint32_t now = millis();
    pollSerial();
    if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
        lastSensorMs = now;
        readMotorEncoder();
        sampleAS5600();
    }
    if (now - lastAdcMs >= ADC_PERIOD_MS) {
        lastAdcMs = now;
        sampleCurrent();
    }
    if (currentSensor.calibrating &&
        now - currentSensor.calibrationStart >= CURRENT_ZERO_TIME_MS) {
        finishCurrentCalibration();
    }
    if (now - lastMotorMs >= MOTOR_UPDATE_MS) {
        lastMotorMs = now;
        updateMotor();
    }
    reportTelemetry();
    delay(1);
}
