#!/usr/bin/env python3
"""Integration test against a running isolated Godot world (never real hardware)."""
import argparse, json, math, socket, time, pathlib, subprocess
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--test-disconnect", action="store_true", help="Explicitly test command loss (disabled by default)")
args = parser.parse_args()
if subprocess.run(['systemctl','is-active','--quiet','sunray-sim3d'], check=False).returncode == 0:
 raise SystemExit('Stop sunray-sim3d before testing the world; concurrent controllers are unsupported.')
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.connect(('127.0.0.1',19091));s.settimeout(.3)
seq=0;reset=1
scene=json.loads((pathlib.Path(__file__).parent/'map.json').read_text());a,b=scene['dockpoints'][-2:]
yaw=math.atan2(b['Y']-a['Y'],b['X']-a['X']);x=b['X']-math.cos(yaw)*2.5;y=b['Y']-math.sin(yaw)*2.5

def command(left=0,right=0,flags=0):
 global seq
 seq+=1;s.send(f'S3D1 {seq} {left} {right} 0 .48 1346.7 {flags} {reset} {x} {y} {yaw} 0 0 0 0'.encode())
 while True:
  p=s.recv(1024).decode().split()
  if int(p[1])==seq:return list(map(float,p[2:]))

def drive(left,right,seconds,flags=0):
 end=time.monotonic()+seconds;v=None
 while time.monotonic()<end:v=command(left,right,flags);time.sleep(.04)
 return v

start=drive(0,0,.3);end=drive(.2,.2,2)
assert .3<math.hypot(end[0]-start[0],end[1]-start[1])<.55,(start,end)
start=drive(0,0,.2);end=drive(.2,.2,1,1)
assert math.hypot(end[0]-start[0],end[1]-start[1])<.02
assert end[3]>start[3]+100,(start,end)
end=drive(-.1,.1,1)
assert math.atan2(math.sin(end[2]-yaw),math.cos(end[2]-yaw))>.25,(end,yaw)
# Optional fault injection; normal tests keep sending commands.
if args.test_disconnect:
 time.sleep(1.2)  # Include the 500 ms status-file publication interval.
 status=json.loads(pathlib.Path('/run/sunray-sim3d/status.json').read_text())
 assert abs(status['linear'])<.001 and abs(status['angular'])<.001,status
reset+=1;drive(0,0,.3)
p=pathlib.Path('/run/sunray-sim3d/camera.frame').read_bytes()
assert p[:4]==b'SRF1' and p[20:22]==b'\xff\xd8'
assert time.time()*1000-int.from_bytes(p[12:20],'little')<500
print('PASS: motion, encoder slip, yaw, fresh JPEG frame' + (', command watchdog' if args.test_disconnect else ' (disconnect test disabled)'))
