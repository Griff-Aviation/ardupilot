#pragma once

#include <stdint.h>

// Vehicle-independent detector/transaction. No HAL, MAVLink, or actuator side effects.
class PayloadRelease {
public:
    static constexpr uint8_t MOTOR_COUNT = 32;
    static constexpr uint16_t COMMAND = 6220;
    struct Config {
        int8_t enable = 0;
        int16_t pwm = 1950;
        float time_s = 0.5f;
        float angle_pct = 110.0f;
        int8_t action = 0;
        int8_t mode = 0;
        int16_t sysid = 1;
        int16_t compid = 25;
        bool valid() const;
    };
    struct Input {
        bool flying = false;
        bool eligible = false;
        float tilt_deg = 0;
        float limit_deg = 0;
        float error_deg = 0;
        uint32_t enabled_motors = 0;
        uint16_t pwm[MOTOR_COUNT] {};
    };
    enum class State : uint8_t {
        Disabled, Monitoring, Saturating, WarnLatched, Pending,
        Accepted, Rejected, TimedOut, Cancelled, InvalidConfig
    };
    enum Event : uint16_t {
        SaturationStarted = 1U << 0, Triggered = 1U << 1, Sent = 1U << 2,
        AckAccepted = 1U << 3, AckRejected = 1U << 4, Timeout = 1U << 5,
        Cancelled = 1U << 6, BadConfig = 1U << 7
    };

    void arm(const Config &config);
    void disarm();
    void update(uint32_t now_ms, const Input &input);
    bool wants_send(uint32_t now_ms) const;
    // Returns true only for the first successful transmission (mode-change boundary).
    bool sent(uint32_t now_ms, bool success);
    void ack(uint32_t now_ms, uint8_t sysid, uint8_t compid, uint16_t command,
             uint8_t result, uint8_t target_sysid, uint8_t target_compid,
             uint8_t local_sysid, uint8_t local_compid);
    void manual_close(uint8_t sysid, uint8_t compid);
    uint16_t take_events() { const uint16_t e = _events; _events = 0; return e; }
    const Config &config() const { return _config; }
    State state() const { return _state; }
    bool active() const { return _armed && _config.enable == 1 && _config.valid(); }
    bool latched() const { return _latched; }
    uint8_t transmissions() const { return _transmissions; }
    uint8_t ack_result() const { return _ack_result; }
    uint8_t trigger_motor() const { return _trigger_motor; } // one-based; zero = none
    uint32_t saturation_ms() const { return _saturation_ms; }
    float error_rate() const { return _error_rate; }
    float tilt_limit() const { return _tilt_limit; }

private:
    struct Motor {
        uint32_t start_ms = 0;
        bool active = false;
        bool loss_seen = false;
    } _motors[MOTOR_COUNT];
    struct Sample { uint32_t ms = 0; float error = 0; } _history[12];
    Config _config;
    State _state = State::Disabled;
    bool _armed = false;
    bool _latched = false;
    bool _tried_send = false;
    bool _in_progress = false;
    uint8_t _history_count = 0;
    uint8_t _history_next = 0;
    uint8_t _transmissions = 0;
    uint8_t _ack_result = 255;
    uint8_t _trigger_motor = 0;
    uint16_t _events = 0;
    uint32_t _trigger_ms = 0;
    uint32_t _last_send_ms = 0;
    uint32_t _last_try_ms = 0;
    uint32_t _saturation_ms = 0;
    float _error_rate = 0;
    float _tilt_limit = 0;
    void reset_detection();
    bool trend(uint32_t now_ms, float error);
    void expire(uint32_t now_ms);
    void cancel();
};
