#!/usr/bin/env python3
"""Closed-loop RTL-entry regression; run on Linux with pymavlink installed.

Use the same runner/model with --label baseline, then --expect-fixed --compare
baseline/summary.json. The low-drag model is a reproducible synthetic fixture,
not a validated model of the flight-log aircraft. RTLB fields use SI units and
the controller's own Dt; PSC/telemetry alone cannot prove a 400 Hz jerk bound.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time

from pymavlink import mavutil

PARAMS = {
    'RTL_SPEED': 500, 'WPNAV_ACCEL': 250, 'WPNAV_JERK': 1,
    'WPNAV_SPEED': 1500, 'LOIT_SPEED': 1500, 'LOIT_ANG_MAX': 25,
    'LOIT_ACC_MAX': 250, 'LOIT_BRK_ACCEL': 50, 'LOIT_BRK_JERK': 330,
    'LOIT_BRK_DELAY': .7, 'PSC_POSXY_P': .66, 'PSC_VELXY_P': 1.6,
    'PSC_VELXY_I': .02, 'PSC_VELXY_D': .9, 'PSC_JERK_XY': 5,
    'ANGLE_MAX': 3000, 'PSC_ANGLE_MAX': 0, 'RTL_ALT': 5000,
    'RTL_CLIMB_MIN': 0, 'RTL_CONE_SLOPE': 0, 'RTL_LOIT_TIME': 1000,
    'SIM_WIND_SPD': 0, 'BATT_FS_LOW_ACT': 0, 'BATT_FS_CRT_ACT': 0,
    # Exclude fast PID/IMU streams so the per-update final-output trace is lossless.
    'FS_GCS_ENABLE': 0, 'FS_THR_ENABLE': 0, 'LOG_BITMASK': 1917,
    'LOG_FILE_BUFSIZE': 200, 'LOG_DISARMED': 0,
    'RC1_DZ': 20, 'RC2_DZ': 20,
}


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')


class Vehicle:
    def __init__(self, args, folder):
        self.args, self.folder = args, folder
        self.t, self.latest, self.acks, self.params, self.samples = 0., {}, {}, {}, []
        self.rc = [1500, 1500, 1000, 1500, 0, 0, 0, 0]
        self.last_send = 0
        folder.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.model, folder/'model.json')
        # Logger buffer size is allocated at startup; setting it after boot is too late.
        (folder/'test.parm').write_text(''.join('%s %.17g\n' % item for item in PARAMS.items()),encoding='ascii')
        self.console = (folder/'sitl.txt').open('w')
        cmd = [str(args.binary), '-S', '-w', '-I', str(args.instance),
               '--model', 'quad:model.json', '--speedup', str(args.speedup),
               '--home', '-35.362938,149.165085,584,0',
               '--defaults', str(args.root/'Tools/autotest/default_params/copter.parm')+','+str(folder/'test.parm')]
        self.process = subprocess.Popen(cmd, cwd=folder, stdout=self.console, stderr=subprocess.STDOUT)
        save(folder/'launch.json', {'command':cmd, 'parameters':PARAMS, 'exercise_exit':args.exercise_exit})
        deadline = time.monotonic()+30
        while True:
            try:
                self.m = mavutil.mavlink_connection('tcp:127.0.0.1:%u' % (5760+10*args.instance), source_system=255)
                break
            except OSError:
                if self.process.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('SITL did not open telemetry; see sitl.txt')
                time.sleep(.2)
        self.m.setup_logfile(str(folder/'flight.tlog'))
        self.wait(lambda: 'HEARTBEAT' in self.latest, 30, 'heartbeat')
        self.m.target_system, self.m.target_component = 1, 1
        self.m.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        for msgid in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                      mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                      mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msgid, 50000)

    def close(self):
        if hasattr(self,'process'):
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait()
        if hasattr(self,'m'):self.m.close()
        if hasattr(self,'console'):self.console.close()
        save(self.folder/'telemetry.json', self.samples)

    def pump(self):
        if time.monotonic()-self.last_send > .08:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.m.mav.rc_channels_override_send(1, 1, *self.rc)
            self.last_send = time.monotonic()
        msg = self.m.recv_match(blocking=True, timeout=.05)
        if msg is None:
            if self.process.poll() is not None: raise RuntimeError('SITL exited')
            return
        typ = msg.get_type()
        if hasattr(msg, 'time_boot_ms'): self.t = max(self.t, msg.time_boot_ms*.001)
        self.latest[typ] = msg
        if typ == 'COMMAND_ACK': self.acks[msg.command] = msg.result
        if typ == 'PARAM_VALUE': self.params[msg.param_id.rstrip('\0')] = msg.param_value
        if typ == 'LOCAL_POSITION_NED':
            a = self.latest.get('ATTITUDE')
            g = self.latest.get('GLOBAL_POSITION_INT')
            self.samples.append(dict(t=self.t, mode=self.latest['HEARTBEAT'].custom_mode,
                                     speed=math.hypot(msg.vx,msg.vy), vx=msg.vx, vy=msg.vy,
                                     x=msg.x,y=msg.y,z=msg.z,
                                     pitch=math.degrees(a.pitch) if a else 0,
                                     roll=math.degrees(a.roll) if a else 0,
                                     altitude=g.relative_alt*.001 if g else -msg.z))

    def wait(self, predicate, timeout, description):
        t0, wall = self.t, time.monotonic()
        while not predicate():
            if self.t-t0>timeout or time.monotonic()-wall>max(30, timeout/self.args.speedup*6):
                raise RuntimeError('Timed out: '+description)
            self.pump()

    def delay(self, seconds):
        until=self.t+seconds
        self.wait(lambda:self.t>=until, seconds+5, 'delay')

    def command(self, command, *params):
        self.acks.pop(command,None)
        self.m.mav.command_long_send(1,1,command,0,*(list(params)+[0]*(7-len(params))))
        self.wait(lambda:command in self.acks,10,'command %u ACK' % command)
        if self.acks[command] != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError('Command %u rejected: %u' % (command,self.acks[command]))

    def parameter(self,name,value):
        self.params.pop(name,None)
        self.m.mav.param_set_send(1,1,name.encode(),value,mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.wait(lambda:name in self.params,10,'parameter '+name)
        if abs(self.params[name]-value)>max(.001,abs(value)*1e-5):
            raise RuntimeError('Parameter mismatch: '+name)

    def mode(self,name):
        number=self.m.mode_mapping()[name]
        self.m.mav.set_mode_send(1,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,number)
        self.wait(lambda:self.latest['HEARTBEAT'].custom_mode==number,10,'mode '+name)

    def takeoff(self):
        for name,value in PARAMS.items(): self.parameter(name,value)
        self.wait(lambda:'GPS_RAW_INT' in self.latest and self.latest['GPS_RAW_INT'].fix_type>=3
                  and 'EKF_STATUS_REPORT' in self.latest and self.latest['EKF_STATUS_REPORT'].flags&16,
                  90,'absolute-position EKF')
        self.mode('GUIDED')
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,30)
        self.wait(lambda:self.samples and self.samples[-1]['altitude']>=29,90,'30 m takeoff')
        self.rc[2]=1500
        self.delay(5)

    def mission(self,bearing):
        home=self.latest['GLOBAL_POSITION_INT']
        north,east=2000*math.cos(math.radians(bearing)),2000*math.sin(math.radians(bearing))
        lat=home.lat+int(north/111319.5*1e7)
        lon=home.lon+int(east/(111319.5*math.cos(math.radians(home.lat*1e-7)))*1e7)
        self.m.mav.mission_clear_all_send(1,1)
        self.delay(.5)
        self.m.mav.mission_count_send(1,1,2)
        deadline=self.t+20
        sent=set()
        while len(sent)<2:
            self.pump()
            req=self.latest.pop('MISSION_REQUEST_INT',None) or self.latest.pop('MISSION_REQUEST',None)
            if req:
                seq=req.seq
                self.m.mav.mission_item_int_send(1,1,seq,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,1,0,0,0,0,
                    home.lat if seq==0 else lat,home.lon if seq==0 else lon,30)
                sent.add(seq)
            if self.t>deadline: raise RuntimeError('Mission upload timed out')
        self.delay(.5)
        self.m.mav.mission_set_current_send(1,1,1)

    def reach_speed(self,profile,bearing,reuse_mission=False):
        if profile=='Auto':
            if not reuse_mission:self.mission(bearing)
            self.mode('AUTO')
        else:
            self.mode('LOITER')
            # Invert the firmware's tan(angle*stick) mapping and deadzone so a
            # diagonal requests the same full thrust magnitude as straight flight.
            angle=math.radians(PARAMS['LOIT_ANG_MAX'])
            for axis,unit in ((0,math.sin(math.radians(bearing))),(1,-math.cos(math.radians(bearing)))):
                normalized=math.atan(math.tan(angle)*unit)/angle
                delta=math.copysign(20+480*abs(normalized),normalized) if abs(normalized)>1e-9 else 0
                self.rc[axis]=round(1500+delta)
        stable=[None]
        def ready():
            if not self.samples:return False
            if self.t-self.samples[-1]['t']<=.25 and 14.8<=self.samples[-1]['speed']<=15.2:
                if stable[0] is None:stable[0]=self.t
            else:stable[0]=None
            return stable[0] is not None and self.t-stable[0]>=3
        self.wait(ready,150,profile+' stable 15 m/s')


def inspect_log(folder,expect_fixed):
    logs=sorted((folder/'logs').glob('*.BIN'),key=lambda p:p.stat().st_mtime)
    if not logs:raise RuntimeError('No DataFlash log produced')
    reader=mavutil.mavlink_connection(str(logs[-1]))
    count=0;peak=0.;jerk_excess=0.;cap_excess=0.;peak_pitch=0.;phase_start=None;phase_end=None
    modes=[];psc={};peak_psc=0.;rtl_start=None;first_target_speed=None;errors=[]
    episodes=[];episode=None;mode_epoch=0;last_trace=None;entries=[]
    first_sequence=None;last_sequence=None;sequence_gaps=0;missing_sequence=0;drop_count=0;disarm_time=None
    while True:
        m=reader.recv_match(type=['RTLB','MODE','ANG','PSCN','PSCE','DSF','ARM'])
        if m is None:break
        typ=m.get_type()
        if typ=='DSF':drop_count=max(drop_count,m.Dp)
        if typ=='ARM' and m.ArmState==0:disarm_time=m.TimeUS*1e-6
        if typ=='MODE':
            modes.append((m.TimeUS*1e-6,m.ModeNum))
            mode_epoch+=1
            if m.ModeNum==6 and rtl_start is None:rtl_start=m.TimeUS*1e-6
            if m.ModeNum==6:entries.append(dict(start_s=m.TimeUS*1e-6,first_PSC_target_speed_ms=None))
        if typ=='ANG' and modes and modes[-1][1]==6:peak_pitch=max(peak_pitch,abs(m.Pitch))
        if typ in ('PSCN','PSCE'):
            psc[typ]=m
            if len(psc)==2 and modes and modes[-1][1]==6:
                peak_psc=max(peak_psc,math.hypot(psc['PSCN'].TAN,psc['PSCE'].TAE))
                n,e=psc['PSCN'],psc['PSCE']
                if first_target_speed is None and min(n.TimeUS,e.TimeUS)*1e-6>=rtl_start and abs(n.TimeUS-e.TimeUS)<10000:
                    first_target_speed=math.hypot(n.TVN,e.TVE)
                if entries and entries[-1]['first_PSC_target_speed_ms'] is None and min(n.TimeUS,e.TimeUS)*1e-6>=entries[-1]['start_s'] and abs(n.TimeUS-e.TimeUS)<10000:
                    entries[-1]['first_PSC_target_speed_ms']=math.hypot(n.TVN,e.TVE)
        if typ!='RTLB':continue
        if not modes or modes[-1][1]!=6:errors.append('RTL limiter remained active outside RTL')
        if not all(math.isfinite(getattr(m,n)) for n in ['Dt','RawN','RawE','OutN','OutE','PrevN','PrevE','Accel','Jerk']):
            errors.append('Non-finite RTLB trace');continue
        if hasattr(m,'Seq'):
            if first_sequence is None:first_sequence=m.Seq
            if last_sequence is not None and m.Seq!=last_sequence+1:sequence_gaps+=1
            last_sequence=m.Seq
        else:missing_sequence+=1
        count+=1;phase_start=phase_start or m.TimeUS*1e-6;phase_end=m.TimeUS*1e-6
        if expect_fixed and not (0<m.Accel<=2.5001 and 0<m.Jerk<=1.0001 and m.Dt>0):
            errors.append('RTLB limits do not match the configured acceleration/jerk')
        if episode is None or episode['mode_epoch']!=mode_epoch:
            episode=dict(mode_epoch=mode_epoch,start_s=m.TimeUS*1e-6,end_s=m.TimeUS*1e-6,
                         samples=0,sum_dt_s=0.,min_dt_s=m.Dt,max_dt_s=m.Dt,
                         max_gap_s=0.,gap_count_over_20ms=0,max_prev_output_difference_mss=0.,
                         sequence_gaps=0,has_sequence=hasattr(m,'Seq'))
            episodes.append(episode);last_trace=None
        episode['samples']+=1;episode['sum_dt_s']+=m.Dt;episode['end_s']=m.TimeUS*1e-6
        episode['min_dt_s']=min(episode['min_dt_s'],m.Dt)
        episode['max_dt_s']=max(episode['max_dt_s'],m.Dt)
        if last_trace is not None:
            gap=(m.TimeUS-last_trace.TimeUS)*1e-6
            episode['max_gap_s']=max(episode['max_gap_s'],gap)
            episode['gap_count_over_20ms']+=int(gap>.02)
            difference=math.hypot(m.PrevN-last_trace.OutN,m.PrevE-last_trace.OutE)
            episode['max_prev_output_difference_mss']=max(episode['max_prev_output_difference_mss'],difference)
            if hasattr(m,'Seq') and m.Seq!=last_trace.Seq+1:episode['sequence_gaps']+=1
        last_trace=m
        mag=math.hypot(m.OutN,m.OutE);peak=max(peak,mag)
        cap_excess=max(cap_excess,mag-m.Accel)
        jerk_excess=max(jerk_excess,math.hypot(m.OutN-m.PrevN,m.OutE-m.PrevE)-m.Jerk*m.Dt)
    for ep in episodes:
        ep['span_s']=ep['end_s']-ep['start_s']
        ep['observed_hz']=(ep['samples']-1)/ep['span_s'] if ep['span_s']>0 else 0
        ep['sum_dt_over_span']=ep['sum_dt_s']/ep['span_s'] if ep['span_s']>0 else 0
        if expect_fixed:
            if ep['span_s']>30:errors.append('Entry/climb limiter phase exceeded 30 seconds in the test fixture')
            if ep['has_sequence']:
                if ep['sequence_gaps']:errors.append('RTLB sequence gaps: '+str(ep['sequence_gaps']))
            elif abs(ep['sum_dt_s']-ep['span_s'])>max(.05,.02*ep['span_s']):
                errors.append('RTLB Dt coverage disagrees with timestamp span: '+str(ep['sum_dt_over_span']))
            if ep['max_prev_output_difference_mss']>1e-4:
                errors.append('RTLB previous-output chain is discontinuous: '+str(ep['max_prev_output_difference_mss']))
    result=dict(log=str(logs[-1]),trace_samples=count,trace_duration_s=sum(ep['span_s'] for ep in episodes),
                trace_wall_span_s=(phase_end-phase_start) if count else 0,RTL_entries=entries,
                trace_start_s=phase_start,trace_end_s=phase_end,first_PSC_target_speed_ms=first_target_speed,
                peak_final_accel_mss=peak,max_cap_excess_mss=cap_excess,
                max_jerk_step_excess_mss=jerk_excess,peak_psc_accel_mss=peak_psc,peak_actual_pitch_deg=peak_pitch)
    result.update(first_sequence=first_sequence,last_sequence=last_sequence,sequence_gaps=sequence_gaps,
                  missing_sequence_records=missing_sequence,max_DSF_dropped_writes=drop_count)
    result['trace_episodes']=episodes
    if expect_fixed:
        if count<200:errors.append('Too few full-rate RTLB samples: '+str(count))
        if first_sequence!=1 or sequence_gaps or missing_sequence:
            errors.append('RTLB sequence must begin at 1 and remain continuous across all RTL entries')
        if drop_count:errors.append('Logger reported dropped writes: '+str(drop_count))
        if peak>2.5001:errors.append('Final acceleration exceeded the absolute 2.5001 m/s2 test bound')
        if cap_excess>1e-4:errors.append('Final acceleration cap exceeded: '+str(cap_excess))
        if jerk_excess>1e-4:errors.append('Final vector jerk exceeded: '+str(jerk_excess))
        if first_target_speed is None or first_target_speed<14:
            errors.append('Initial speed target dropped below 14 m/s: '+str(first_target_speed))
        for entry in entries:
            if entry['first_PSC_target_speed_ms'] is None or entry['first_PSC_target_speed_ms']<14:
                errors.append('RTL entry speed target dropped below 14 m/s: '+str(entry))
    # Disarm alone is insufficient: a crash or failsafe LAND must not pass as RTL completion.
    telemetry=json.loads((folder/'telemetry.json').read_text())
    if telemetry:
        home=telemetry[0]
        final=min(telemetry,key=lambda sample:abs(sample['t']-disarm_time)) if disarm_time is not None else telemetry[-1]
        distance=math.hypot(final['x']-home['x'],final['y']-home['y'])
        result['completion']=dict(final_mode=modes[-1][1] if modes else None,
                                  distance_from_takeoff_m=distance,altitude_m=final['altitude'],
                                  speed_ms=final['speed'],disarm_logged=disarm_time is not None)
        if not modes or modes[-1][1]!=6 or distance>5 or abs(final['altitude'])>1 or final['speed']>1 or disarm_time is None:
            errors.append('RTL did not finish disarmed, near takeoff, near ground, at low speed, in RTL mode')
        if disarm_time is not None and abs(disarm_time-final['t'])>1:
            errors.append('Completion telemetry is stale relative to disarm')
    else:errors.append('No telemetry available to prove RTL return/landing completion')
    launch=json.loads((folder/'launch.json').read_text())
    expected_entries=2 if launch.get('exercise_exit') else 1
    if len(entries)!=expected_entries:errors.append('Unexpected RTL entry count: '+str(len(entries)))
    result['validation_errors']=sorted(set(errors))
    save(folder/'trace_metrics.json',result)
    return result


def run_case(args,profile,bearing):
    folder=args.output/('%s_%s_%gdeg' % (args.label,profile.lower(),bearing))
    v=None;result=dict(profile=profile,bearing_deg=bearing,status='error')
    try:
        v=Vehicle.__new__(Vehicle)
        v.__init__(args,folder);v.takeoff();v.reach_speed(profile,bearing)
        entry=v.samples[-1].copy();rtl_command_time=v.t
        normal=[s for s in v.samples if rtl_command_time-3<=s['t']<=rtl_command_time]
        result.update(entry=entry,normal_mean={key:sum(s[key] for s in normal)/len(normal)
                                             for key in ('speed','pitch','roll','altitude')})
        v.mode('RTL')
        # Center after mode acknowledgement: do not initiate Loiter braking first.
        v.rc[0]=v.rc[1]=1500
        if args.exercise_exit:
            v.delay(1.5);v.mode('AUTO' if profile=='Auto' else 'LOITER');v.delay(2)
            v.reach_speed(profile,bearing,reuse_mission=True)
            result['reentry']=v.samples[-1].copy()
            v.mode('RTL');v.rc[0]=v.rc[1]=1500
        v.wait(lambda:not v.latest['HEARTBEAT'].base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
               360,'RTL landing and disarm')
        finish_t=v.t
        # Leave the logger IO thread time to flush the final ARM record before SIGTERM.
        v.delay(2)
        result.update(status='passed',RTL_completed=True,elapsed_after_RTL_s=finish_t-rtl_command_time,
                      disarmed_heartbeat_time_s=finish_t,logger_flush_delay_s=2)
    except Exception as exc:
        result['error']=str(exc)
    finally:
        if v is not None:v.close()
    if result['status']=='passed' or any((folder/'logs').glob('*.BIN')):
        try:
            result.update(inspect_log(folder,args.expect_fixed))
            if result['validation_errors']:
                result.update(status='failed',error='; '.join(filter(None,[result.get('error')]+result['validation_errors'])))
        except Exception as exc:
            result.update(status='failed',error='; '.join(filter(None,[result.get('error'),str(exc)])))
    save(folder/'result.json',result)
    print(json.dumps(result),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path)
    parser.add_argument('--inspect-folder',type=Path)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--model',type=Path,default=Path(__file__).parent/'models/rtl-entry-low-drag.json')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--label',default='fixed')
    parser.add_argument('--expect-fixed',action='store_true')
    parser.add_argument('--exercise-exit',action='store_true',help='Abort braking to original mode, regain 15 m/s and re-enter RTL')
    parser.add_argument('--profiles',default='Loiter,Auto')
    parser.add_argument('--bearings',default='0')
    parser.add_argument('--speedup',type=float,default=8)
    parser.add_argument('--instance',type=int,default=42)
    parser.add_argument('--compare',type=Path)
    args=parser.parse_args()
    if args.inspect_folder:
        result=inspect_log(args.inspect_folder.resolve(),args.expect_fixed)
        print(json.dumps(result,indent=2),flush=True)
        raise SystemExit(bool(result['validation_errors']))
    if args.binary is None or args.output is None:parser.error('--binary and --output are required for a flight')
    for key in ('binary','root','model','output'):setattr(args,key,getattr(args,key).resolve())
    args.output.mkdir(parents=True,exist_ok=True)
    results=[run_case(args,p,float(b)) for p in args.profiles.split(',') for b in args.bearings.split(',')]
    comparisons=[]
    if args.compare:
        previous=json.loads(args.compare.read_text())['cases']
        for result in results:
            old=next((p for p in previous if (p['profile'],p['bearing_deg'])==(result['profile'],result['bearing_deg'])),None)
            if not old or old.get('status')!='passed' or 'normal_mean' not in result:
                comparisons.append(dict(profile=result['profile'],bearing_deg=result['bearing_deg'],passed=False,error='Missing successful comparable flight'))
                continue
            if old and 'normal_mean' in old and 'normal_mean' in result:
                delta={k:result['normal_mean'][k]-old['normal_mean'][k] for k in old['normal_mean']}
                ok=abs(delta['speed'])<.25 and abs(delta['pitch'])<1 and abs(delta['roll'])<1 and abs(delta['altitude'])<.5
                comparisons.append(dict(profile=result['profile'],bearing_deg=result['bearing_deg'],delta=delta,passed=ok))
    passed=all(r['status']=='passed' for r in results) and all(c['passed'] for c in comparisons)
    save(args.output/'summary.json',dict(label=args.label,passed=passed,cases=results,normal_flight_comparison=comparisons))
    raise SystemExit(0 if passed else 1)


if __name__=='__main__':main()
