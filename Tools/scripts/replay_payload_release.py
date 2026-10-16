#!/usr/bin/env python3
"""Replay DataFlash logs through the production payload detector (read-only, no vehicle I/O)."""
from pathlib import Path
import argparse
from contextlib import nullcontext
import json
import os
import shutil
import subprocess
import tempfile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs',nargs='+',type=Path)
    parser.add_argument('--cxx',default=os.environ.get('CXX','g++'))
    parser.add_argument('--angle-max-only',action='store_true',help='Comparison only: ignore Loiter-specific angle limit')
    parser.add_argument('--output',type=Path,help='JSON results; input files are never modified')
    parser.add_argument('--build-dir',type=Path,help='Keep the replay executable in this directory')
    args=parser.parse_args()
    repo=Path(__file__).resolve().parents[2]
    compiler=shutil.which(args.cxx) or args.cxx
    env=os.environ.copy()
    env['PATH']=str(Path(compiler).parent)+os.pathsep+env.get('PATH','')
    results=[]
    manager=(nullcontext(args.build_dir) if args.build_dir else
             tempfile.TemporaryDirectory(prefix='ardupilot-pldr-replay-'))
    with manager as directory:
        Path(directory).mkdir(parents=True,exist_ok=True)
        exe=Path(directory)/('replay.exe' if os.name=='nt' else 'replay')
        subprocess.run([compiler,'-std=c++11','-O2','-Wall','-Wextra','-Werror',
                        '-I'+str(repo/'ArduCopter'),str(repo/'ArduCopter/PayloadRelease.cpp'),
                        str(repo/'ArduCopter/tests/replay_payload_release.cpp'),'-o',str(exe)],
                       env=env,check=True)
        for log in args.logs:
            command=[str(exe),str(log.resolve())]
            if args.angle_max_only:
                command.append('--angle-max-only')
            result=subprocess.run(command,env=env,text=True,capture_output=True)
            if result.returncode:
                raise SystemExit(result.stderr)
            report=json.loads(result.stdout)
            results.append(report)
            print(json.dumps(report),flush=True)
    if args.output:
        args.output.write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':
    main()
