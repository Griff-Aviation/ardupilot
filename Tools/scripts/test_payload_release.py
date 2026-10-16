#!/usr/bin/env python3
"""Compile/run the real payload core and production integration at stubbed I/O boundaries."""
from pathlib import Path
import argparse
import os
import shutil
import subprocess
import tempfile

def function(source, marker):
    start=source.index(marker)
    p=source.index('{',start)+1
    depth=1
    while depth:
        depth+=(source[p]=='{')-(source[p]=='}')
        p+=1
    return source[start:p]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cxx',default=os.environ.get('CXX','g++'))
    parser.add_argument('--mavlink-include',type=Path,help='Directory containing ardupilotmega/mavlink.h')
    parser.add_argument('--build-dir',type=Path)
    args=parser.parse_args()
    repo=Path(__file__).resolve().parents[2]
    include=args.mavlink_include or repo/'build/sitl/libraries/GCS_MAVLink/include/mavlink/v2.0'
    if not (include/'ardupilotmega/mavlink.h').is_file():
        include=repo/'libraries/GCS_MAVLink/include/mavlink/v2.0'
    if not (include/'ardupilotmega/mavlink.h').is_file():
        parser.error('Generate MAVLink headers with waf configure/build or supply --mavlink-include.')
    build=args.build_dir or Path(tempfile.mkdtemp(prefix='ardupilot-pldr-tests-'))
    build.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy()
    compiler=shutil.which(args.cxx) or args.cxx
    env['PATH']=str(Path(compiler).parent)+os.pathsep+env.get('PATH','')
    suffix='.exe' if os.name=='nt' else ''
    logs=[]
    def run(command):
        result=subprocess.run([str(x) for x in command],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        logs.extend([' '.join(str(x) for x in command),result.stdout])
        (build/'tests.log').write_text('\n'.join(logs))
        print(result.stdout,end='',flush=True)
        if result.returncode:
            raise SystemExit(result.returncode)
    flags=['-std=c++11','-O2','-Wall','-Wextra','-Wshadow','-Wformat-security','-Werror','-I'+str(repo/'ArduCopter')]
    exe=build/('core'+suffix)
    run([compiler,*flags,repo/'ArduCopter/PayloadRelease.cpp',repo/'ArduCopter/tests/test_payload_release.cpp','-o',exe])
    run([exe])
    source=(repo/'ArduCopter/payload_release.cpp').read_text()
    methods='\n'.join(function(source,m) for m in (
        'PayloadRelease::Config Copter::payload_release_config(',
        'void Copter::payload_release_check(',
        'void Copter::payload_release_handle_message('))
    route=(repo/'libraries/GCS_MAVLink/MAVLink_routing.cpp').read_text()
    methods+='\n'+function(route,'bool MAVLink_routing::send_to_component(')
    loiter=repo/'libraries/AC_WPNav/AC_Loiter.cpp'
    if loiter.is_file():
        loiter_source=loiter.read_text()
    else:
        loiter_source=subprocess.check_output(['git','show','HEAD:libraries/AC_WPNav/AC_Loiter.cpp'],cwd=repo,text=True)
    methods+='\n'+function(loiter_source,'float AC_Loiter::get_angle_max_rad(')
    fixture=(repo/'ArduCopter/tests/payload_release_integration.cpp.in').read_text()
    generated=build/'integration.cpp'
    generated.write_text(fixture.replace('// @PRODUCTION_METHODS@',methods))
    exe=build/('integration'+suffix)
    run([compiler,*flags,'-Wno-address-of-packed-member','-isystem',include,
         repo/'ArduCopter/PayloadRelease.cpp',generated,'-o',exe])
    run([exe])
    print('BUILD_DIR='+str(build))

if __name__=='__main__':
    main()
