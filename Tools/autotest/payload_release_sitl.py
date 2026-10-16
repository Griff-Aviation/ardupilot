"""Closed-loop SITL scenario; requires the ArduCopter autotest harness.

The heavy quad is synthetic, not a validated model of a G30. This test deliberately
removes one simulated motor's thrust. Its B100 peer exists only on the SITL link.
"""
import os

from pymavlink import mavutil
from vehicle_test_suite import NotAchievedException


def run(test):
    model = os.path.join(test.rootdir(), 'Tools', 'autotest', 'models',
                         'payload-release-heavy.json')
    test.context_push()
    try:
        for response in ('warn', 'accept', 'reject', 'timeout', 'manual-close'):
            test.start_subtest('Automatic payload release: ' + response)
            test.customise_SITL_commandline([], model='quad:' + model)
            test.context_push()
            state = {'commands': [], 'triggers': [], 'crash': None}
            try:
                test.context_collect('STATUSTEXT')
                test.set_parameters({
                    'PLDR_ENABLE': 1,
                    'PLDR_ACTION': 0 if response == 'warn' else 1,
                    'PLDR_MODE': 2,
                    'PLDR_PWM': 1950,
                    'PLDR_TIME': 0.5,
                    'PLDR_ANG_PCT': 110,
                    'PLDR_SYSID': 1,
                    'PLDR_COMPID': 25,
                    'LOIT_ANG_MAX': 25,
                    'ANGLE_MAX': 3000,
                    'MOT_PWM_MIN': 1000,
                    'MOT_PWM_MAX': 2000,
                    'MOT_THST_EXPO': 0.8,
                    'SIM_ENGINE_FAIL': 0,
                    'SIM_ENGINE_MUL': 0,
                    'BATT_FS_LOW_ACT': 0,
                    'BATT_FS_CRT_ACT': 0,
                    'FS_CRASH_CHECK': 1,
                })

                def as_b100(send):
                    mav = test.mav.mav
                    previous = mav.srcSystem, mav.srcComponent
                    try:
                        mav.srcSystem, mav.srcComponent = 1, 25
                        send(mav)
                    finally:
                        mav.srcSystem, mav.srcComponent = previous

                def peer_hook(_mav, message):
                    if message.get_type() == 'STATUSTEXT':
                        if 'PLDR: Trigger' in message.text:
                            state['triggers'].append(test.get_sim_time_cached())
                        if 'Crash: Disarming' in message.text:
                            state['crash'] = test.get_sim_time_cached()
                        return
                    if message.get_type() != 'COMMAND_LONG' or message.command != 6220:
                        return
                    if message.target_system != 1 or message.target_component != 25:
                        raise NotAchievedException('Wrong release recipient')
                    if message.param1 != 1 or any(getattr(message, 'param%u' % i) != 0
                                                  for i in range(2, 8)):
                        raise NotAchievedException('Wrong release command parameters')
                    state['commands'].append((test.get_sim_time_cached(), message.confirmation))
                    if response in ('accept', 'reject'):
                        result = (mavutil.mavlink.MAV_RESULT_ACCEPTED if response == 'accept'
                                  else mavutil.mavlink.MAV_RESULT_DENIED)
                        as_b100(lambda mav: mav.command_ack_send(
                            6220, result, 0, 0, message.get_srcSystem(), message.get_srcComponent()))
                    elif response == 'manual-close' and len(state['commands']) == 1:
                        test.mav.mav.command_long_send(1, 25, 6220, 0, 0, 0, 0, 0, 0, 0, 0)

                test.install_message_hook_context(peer_hook)
                test.takeoff(80, mode='LOITER')
                # Learn the peer route before injecting a failure. Source identity is
                # restored immediately, preserving the autotest GCS identity.
                as_b100(lambda mav: mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE))
                test.delay_sim_time(2)
                if state['triggers'] or state['commands']:
                    raise NotAchievedException('Unexpected trigger in normal flight')
                if response == 'warn':
                    # The arming snapshot must retain warn-only action in this flight.
                    test.set_parameter('PLDR_ACTION', 1)
                test.set_parameter('SIM_ENGINE_FAIL', 4)
                test.wait_statustext('PLDR: Trigger', timeout=8, check_context=True)
                test.set_parameter('SIM_ENGINE_FAIL', 0)
                test.delay_sim_time(1.2)
                expected = {'warn': 0, 'accept': 1, 'reject': 1,
                            'timeout': 3, 'manual-close': 1}[response]
                if len(state['commands']) != expected or len(state['triggers']) != 1:
                    raise NotAchievedException('Unexpected release/retry/latch count: %s' % state)
                if state['crash'] is not None:
                    first = state['commands'][0][0] if state['commands'] else state['triggers'][0]
                    if state['crash'] <= first:
                        raise NotAchievedException('Crash disarm preceded release trigger/send')
                if response != 'warn':
                    test.wait_mode('ALT_HOLD')
                elif test.mav.flightmode != 'LOITER':
                    raise NotAchievedException('Warn-only action changed flight mode')
                if response == 'timeout' and [c[1] for c in state['commands']] != [0, 1, 2]:
                    raise NotAchievedException('Incorrect command confirmation sequence')
                # Every new scenario arms a fresh vehicle: the next trigger also
                # checks reset on disarm/reboot and adoption of new settings.
            finally:
                test.set_parameter('SIM_ENGINE_FAIL', 0)
                if test.armed():
                    test.disarm_vehicle(force=True)
                test.context_pop()
    finally:
        test.context_pop()
