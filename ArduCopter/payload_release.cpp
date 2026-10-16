#include "Copter.h"

PayloadRelease::Config Copter::payload_release_config() const
{
    PayloadRelease::Config config;
    config.enable = g2.pldr_enable.get();
    config.pwm = g2.pldr_pwm.get();
    config.time_s = g2.pldr_time.get();
    config.angle_pct = g2.pldr_angle_pct.get();
    config.action = g2.pldr_action.get();
    config.mode = g2.pldr_mode.get();
    config.sysid = g2.pldr_sysid.get();
    config.compid = g2.pldr_compid.get();
    return config;
}

// Runs immediately before crash_check, after motor mixing and attitude update.
void Copter::payload_release_check()
{
    if (!motors->armed()) {
        payload_release_notice_sent = false;
    }
    const uint32_t now_ms = AP_HAL::millis();
    PayloadRelease::Input input;
    input.flying = motors->armed() && !ap.land_complete;
    input.eligible = !standby_active && flightmode->crash_check_enabled() &&
        flightmode->mode_number() != Mode::Number::TURTLE;
#if FRAME_CONFIG == HELI_FRAME
    input.eligible = false; // this implementation observes multicopter motor PWM
#endif
    uint16_t highest_pwm = 0;
    uint8_t highest_motor = 0;
    uint8_t highest_channel = 0;
    bool mode_failed = false;
    if (payload_release.active()) {
        input.tilt_deg = degrees(acosf(constrain_float(ahrs.cos_roll() * ahrs.cos_pitch(), -1, 1)));
        input.error_deg = attitude_control->get_att_error_angle_deg();
        input.limit_deg = aparm.angle_max.get() * 0.01f;
#if MODE_LOITER_ENABLED
        if (flightmode->mode_number() == Mode::Number::LOITER) {
            input.limit_deg = degrees(loiter_nav->get_angle_max_rad());
        }
#endif
        static_assert(AP_MOTORS_MAX_NUM_MOTORS <= PayloadRelease::MOTOR_COUNT, "Payload motor array too small");
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (!motors->is_motor_enabled(i)) {
                continue;
            }
            const auto function = SRV_Channels::get_motor_function(i);
            if (!SRV_Channels::get_output_pwm(function, input.pwm[i])) {
                continue;
            }
            input.enabled_motors |= uint32_t(1) << i;
            if (input.pwm[i] > highest_pwm) {
                highest_pwm = input.pwm[i];
                highest_motor = i + 1;
                uint8_t channel;
                if (SRV_Channels::find_channel(function, channel)) {
                    highest_channel = channel + 1;
                }
            }
        }
    }
    payload_release.update(now_ms, input);

#if HAL_GCS_ENABLED
    if (input.flying && input.eligible && payload_release.wants_send(now_ms)) {
        const auto &config = payload_release.config();
        mavlink_command_long_t command {};
        command.target_system = config.sysid;
        command.target_component = config.compid;
        command.command = PayloadRelease::COMMAND;
        command.param1 = 1;
        command.confirmation = payload_release.transmissions();
        const bool sent = GCS_MAVLINK::send_to_component(config.sysid, config.compid,
            MAVLINK_MSG_ID_COMMAND_LONG, reinterpret_cast<const char *>(&command), sizeof(command));
        const bool first_send = payload_release.sent(now_ms, sent);
        if (first_send && config.mode == 2 && !set_mode(Mode::Number::ALT_HOLD, ModeReason::PAYLOAD_RELEASE)) {
            mode_failed = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PLDR: AltHold request failed");
        }
    }
#endif

    const uint16_t events = payload_release.take_events() | (mode_failed ? 256U : 0U);
    if ((events & PayloadRelease::SaturationStarted) &&
        (!payload_release_notice_sent || now_ms - payload_release_notice_ms >= 5000)) {
        payload_release_notice_sent = true;
        payload_release_notice_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PLDR: Motor %u out %u saturation", unsigned(highest_motor), unsigned(highest_channel));
    }
    if (events & PayloadRelease::Triggered) {
        if (payload_release.config().action == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PLDR: Trigger motor %u (warn only)", unsigned(payload_release.trigger_motor()));
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PLDR: Trigger motor %u, release requested", unsigned(payload_release.trigger_motor()));
        }
    }
    if (events & PayloadRelease::Sent) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PLDR: Release command sent (%u/3)", unsigned(payload_release.transmissions()));
    }
    if (events & PayloadRelease::AckAccepted) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PLDR: B100 accepted release command");
    }
    if (events & PayloadRelease::AckRejected) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PLDR: Release rejected, ACK %u", unsigned(payload_release.ack_result()));
    }
    if (events & PayloadRelease::Timeout) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PLDR: Release timeout, %u sends", unsigned(payload_release.transmissions()));
    }
    if (events & PayloadRelease::Cancelled) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PLDR: Release retries cancelled");
    }
    if (events & PayloadRelease::BadConfig) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PLDR: Invalid configuration, inactive");
    }

#if HAL_LOGGING_ENABLED
    const uint8_t state = uint8_t(payload_release.state());
    if ((payload_release.active() && now_ms - payload_release_log_ms >= 50) ||
        events != 0 || state != payload_release_logged_state) {
        payload_release_log_ms = now_ms;
        payload_release_logged_state = state;
        // @LoggerMessage: PLDR
        // @Description: Automatic payload release detector and command transaction
        // @Field: TimeUS: Time since system startup
        // @Field: Tilt: Actual tilt in degrees
        // @Field: Lim: Effective release tilt threshold in degrees
        // @Field: Err: Thrust-direction attitude error in degrees (excludes yaw)
        // @Field: Rate: Change in attitude error over 100 ms, in degrees/second
        // @Field: PWM: Highest enabled motor PWM in microseconds
        // @Field: Mot: One-based motor number with highest PWM, zero if unavailable
        // @Field: Ch: One-based output channel for Mot, zero if unavailable
        // @Field: Sat: Longest current per-motor saturation interval in seconds
        // @Field: St: State: 0 disabled, 1 monitoring, 2 saturating, 3 warn latched, 4 pending, 5 accepted, 6 rejected, 7 timeout, 8 cancelled, 9 invalid config
        // @Field: Ev: Event bits: 0 saturation start, 1 trigger, 2 sent, 3 accepted, 4 rejected, 5 timeout, 6 cancelled, 7 invalid config, 8 AltHold failed
        // @Field: TM: One-based motor responsible for the latched trigger
        // @Field: Tx: Number of successful command transmissions
        // @Field: Ack: Last matching ACK result; 255 if none
        // @Field: Mode: Current flight mode number
        AP::logger().Write("PLDR", "TimeUS,Tilt,Lim,Err,Rate,PWM,Mot,Ch,Sat,St,Ev,TM,Tx,Ack,Mode",
            "QffffHBBfBHBBBB", AP_HAL::micros64(), double(input.tilt_deg),
            double(payload_release.tilt_limit()), double(input.error_deg), double(payload_release.error_rate()),
            unsigned(highest_pwm), unsigned(highest_motor), unsigned(highest_channel), double(payload_release.saturation_ms() * 0.001f),
            unsigned(state), unsigned(events), unsigned(payload_release.trigger_motor()),
            unsigned(payload_release.transmissions()), unsigned(payload_release.ack_result()), unsigned(flightmode->mode_number()));
    }
#endif
}

#if HAL_GCS_ENABLED
void Copter::payload_release_handle_message(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);
        payload_release.ack(AP_HAL::millis(), msg.sysid, msg.compid, ack.command, ack.result,
            ack.target_system, ack.target_component, mavlink_system.sysid, mavlink_system.compid);
    } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t command;
        mavlink_msg_command_long_decode(&msg, &command);
        if (command.command == PayloadRelease::COMMAND && is_zero(command.param1)) {
            payload_release.manual_close(command.target_system, command.target_component);
        }
    }
}
#endif
