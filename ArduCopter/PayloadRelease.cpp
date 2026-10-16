#include "PayloadRelease.h"

#include <cmath>

bool PayloadRelease::Config::valid() const
{
    if (enable == 0) {
        return true;
    }
    return enable == 1 && pwm >= 1000 && pwm <= 2200 &&
        std::isfinite(time_s) && time_s >= 0.1f && time_s <= 2.0f &&
        std::isfinite(angle_pct) && angle_pct >= 100 && angle_pct <= 200 &&
        (action == 0 || action == 1) && (mode == 0 || mode == 2) &&
        sysid >= 1 && sysid <= 255 && compid >= 1 && compid <= 255;
}

void PayloadRelease::arm(const Config &config)
{
    disarm();
    _config = config;
    _armed = true;
    if (!config.valid()) {
        _state = State::InvalidConfig;
        _events |= BadConfig;
    } else if (config.enable == 1) {
        _state = State::Monitoring;
    }
}

void PayloadRelease::disarm()
{
    reset_detection();
    _armed = _latched = _tried_send = _in_progress = false;
    _transmissions = _trigger_motor = 0;
    _ack_result = 255;
    _events = 0;
    _state = State::Disabled;
}

void PayloadRelease::reset_detection()
{
    for (auto &motor : _motors) {
        motor = Motor{};
    }
    _history_count = _history_next = 0;
    _saturation_ms = 0;
    _error_rate = 0;
    _tilt_limit = 0;
}

// Interpolate the attitude error exactly 100 ms ago. Store at >=10 ms spacing
// so twelve samples cover the window without main-loop-rate-dependent storage.
bool PayloadRelease::trend(uint32_t now_ms, float error)
{
    bool ready = false;
    uint32_t older_age = UINT32_MAX;
    uint32_t newer_age = 0;
    float older_error = error;
    float newer_error = error;
    for (uint8_t i = 0; i < _history_count; i++) {
        const uint32_t age = now_ms - _history[i].ms;
        if (age >= 100 && age < older_age) {
            older_age = age;
            older_error = _history[i].error;
            ready = true;
        } else if (age < 100 && age >= newer_age) {
            newer_age = age;
            newer_error = _history[i].error;
        }
    }
    _error_rate = 0;
    if (ready) {
        const float fraction = float(100 - newer_age) / float(older_age - newer_age);
        const float previous = newer_error + fraction * (older_error - newer_error);
        _error_rate = (error - previous) * 10.0f;
    }
    const uint8_t previous_index = (_history_next + 11U) % 12U;
    if (_history_count == 0 || now_ms - _history[previous_index].ms >= 10) {
        _history[_history_next].ms = now_ms;
        _history[_history_next].error = error;
        _history_next = (_history_next + 1U) % 12U;
        if (_history_count < 12) {
            _history_count++;
        }
    }
    return ready;
}

void PayloadRelease::cancel()
{
    if (_state == State::Pending) {
        _state = State::Cancelled;
        _events |= Cancelled;
    }
}

void PayloadRelease::expire(uint32_t now_ms)
{
    if (_state == State::Pending && now_ms - _trigger_ms >= 750) {
        _state = State::TimedOut;
        _events |= Timeout;
    }
}

void PayloadRelease::update(uint32_t now_ms, const Input &input)
{
    if (!active()) {
        return;
    }
    if (!input.flying) {
        cancel();
    }
    expire(now_ms);
    if (!input.eligible || !input.flying || !std::isfinite(input.tilt_deg) ||
        !std::isfinite(input.error_deg) || !std::isfinite(input.limit_deg) || input.limit_deg <= 0) {
        reset_detection();
        if (!_latched) {
            _state = State::Monitoring;
        }
        return;
    }
    _tilt_limit = input.limit_deg * (_config.angle_pct * 0.01f);
    const bool growing = trend(now_ms, input.error_deg) && _error_rate > 0;
    const bool loss = growing && input.tilt_deg > _tilt_limit;
    bool previously_saturated = false;
    for (const auto &motor : _motors) {
        previously_saturated |= motor.active;
    }
    bool saturated = false;
    _saturation_ms = 0;
    const uint32_t required_ms = uint32_t(_config.time_s * 1000.0f + 0.5f);
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor &motor = _motors[i];
        if (!(input.enabled_motors & (uint32_t(1) << i)) || input.pwm[i] < _config.pwm) {
            motor = Motor{};
            continue;
        }
        saturated = true;
        if (!motor.active) {
            motor.active = true;
            motor.start_ms = now_ms;
        }
        motor.loss_seen |= loss;
        const uint32_t elapsed = now_ms - motor.start_ms;
        if (elapsed > _saturation_ms) {
            _saturation_ms = elapsed;
        }
        if (!_latched && elapsed >= required_ms && motor.loss_seen) {
            _latched = true;
            _trigger_motor = i + 1;
            _trigger_ms = now_ms;
            _state = _config.action == 1 ? State::Pending : State::WarnLatched;
            _events |= Triggered;
        }
    }
    if (!_latched) {
        _state = saturated ? State::Saturating : State::Monitoring;
    }
    if (saturated && !previously_saturated && !_latched) {
        _events |= SaturationStarted;
    }
}

bool PayloadRelease::wants_send(uint32_t now_ms) const
{
    return _state == State::Pending && now_ms - _trigger_ms < 750 &&
        !_in_progress && _transmissions < 3 &&
        (!_tried_send || now_ms - _last_try_ms >= 20) &&
        (_transmissions == 0 || now_ms - _last_send_ms >= 250);
}

bool PayloadRelease::sent(uint32_t now_ms, bool success)
{
    if (!wants_send(now_ms)) {
        return false;
    }
    _last_try_ms = now_ms;
    _tried_send = true;
    if (!success) {
        return false;
    }
    _last_send_ms = now_ms;
    _transmissions++;
    _events |= Sent;
    return _transmissions == 1;
}

void PayloadRelease::ack(uint32_t now_ms, uint8_t sysid, uint8_t compid,
                         uint16_t command, uint8_t result, uint8_t target_sysid,
                         uint8_t target_compid, uint8_t local_sysid, uint8_t local_compid)
{
    expire(now_ms);
    if (_state != State::Pending || _transmissions == 0 || command != COMMAND ||
        sysid != _config.sysid || compid != _config.compid ||
        (target_sysid != 0 && target_sysid != local_sysid) ||
        (target_compid != 0 && target_compid != local_compid)) {
        return;
    }
    _ack_result = result;
    if (result == 0) { // MAV_RESULT_ACCEPTED
        _state = State::Accepted;
        _events |= AckAccepted;
    } else if (result == 5) { // MAV_RESULT_IN_PROGRESS: wait, do not duplicate the command
        _in_progress = true;
    } else {
        _state = State::Rejected;
        _events |= AckRejected;
    }
}

void PayloadRelease::manual_close(uint8_t sysid, uint8_t compid)
{
    if (sysid == _config.sysid && compid == _config.compid) {
        cancel();
    }
}
