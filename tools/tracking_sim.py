# simulate robot using RTK-GPS tracking a line (stanley control)

# https://www.youtube.com/watch?v=9Y7kVIJs_JI
# https://github.com/qiaoxu123/Self-Driving-Cars/blob/master/Part1-Introduction_to_Self-Driving_Cars/Module6-Vehicle_Lateral_Control/module-6-vehicle-lateral-control.md


import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


tracklen = 100.0 # 20.0 track length [m]
speed = 0.1 # starting speed [m/s]
maxspeed = 1.0 # 0.5 speed [m/s]
dt = 0.1 # 0.02 control period

noiseGps = 0.0 # 0.1 GPS noise 
noiseProcessYaw = 0.002  # 0.02 process noise

processLatency = 0.999  # 0.99 # process latency (steering latency)

doControl = True
stanley_p = 30.0   # 1.0 # 1.0 stanley angular control gain
stanley_k = 30.0   # 5.0 # stanley lateral control gain # 0.5 (normal), 0.1 (slow)
stanley_ks = 0.001  # smoothness


# scale setangle, so that both PI angles have the same sign
def scalePIangles(setAngle, currAngle):
  if ((setAngle >= math.pi/2) and (currAngle <= -math.pi/2)): return (setAngle-2*math.pi)
  elif ((setAngle <= -math.pi/2) and (currAngle >= math.pi/2)): return (setAngle+2*math.pi)
  else: return setAngle


# rescale to -PI..+PI
def scalePI(v):
  d = v
  while (d < 0): d=d+2*math.pi
  while (d >= 2*math.pi): d=d-2*math.pi
  if (d >= math.pi): return (-2*math.pi+d)
  elif (d < -math.pi): return (2*math.pi+d)
  else: return d


# computes minimum distance between x radiant (current-value) and w radiant (set-value)
def distancePI(x, w):
  # cases:
  # w=330 degree, x=350 degree => -20 degree
  # w=350 degree, x=10  degree => -20 degree
  # w=10  degree, x=350 degree =>  20 degree
  # w=0   degree, x=190 degree => 170 degree
  # w=190 degree, x=0   degree => -170 degree
  d = scalePI(w - x)
  if (d < -math.pi): d = d + 2*math.pi
  elif (d > math.pi): d = d - 2*math.pi
  return d


# compute distance to (infinite) line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points)
def distanceLineInfinite(px, py, x1, y1, x2, y2):
  len = math.sqrt( (y2-y1)**2 + (x2-x1)**2 )
  if (abs(len) < 0.01): return 0
  distToLine = ((y2-y1)*px-(x2-x1)*py+(x2*y1-y2*x1)) / len
  return distToLine

# compute course (angle in rad) between two points
def pointsAngle(x1, y1, x2, y2):
  dX = x2 - x1
  dY = y2 - y1
  angle = scalePI(math.atan2(dY, dX))
  return angle



    
# Data for three-dimensional scattered points
# https://blog.mide.com/accelerometer-specifications-decoding-a-datasheet


# --- goal pos -----------
gx = tracklen
gy = 0.0
gz = 0.0

# --- true data ----------
gxdata= []
gydata= []
gzdata= []
yawdata = []
xdata = []  
ydata = []
zdata = []
mexdata = []
meydata = []
mezdata = []
elateral = []
eangular = []
timedata = []
speeddata = []
ctl_angular_data = []
ctl_lateral_data = []
steering_data = []
yaw = 0 # start orientation
pos = np.array([0.0, 0.0, 0.0])

# ---- measurement data ----
mxdata = [] 
mydata = []
mzdata = []
mpos = np.array([0.0,0.0,0.0])

# ----------------------------
print('speed',speed)
print('dt',dt)
time = 0
#noiseAccHour = 0.5* noiseAcc * 1e-6 * 3600.0**2
#print('noiseAcc [m] per hour', noiseAccHour  )
#print('driftGyro [deg] per hour', biasGyro )

nextSpeedStepTime = 0.0

while (pos[0] < gx-2.0) and (time < 200):         
  percent = pos[0] / gx
  #print(percent, speed)
  if time > nextSpeedStepTime:
    nextSpeedStepTime = time + 20.0
    speed = min(maxspeed, speed + 0.2) 
    print("speed",speed, 'x', pos[0], 'gx', gx)
    
  xdata.append(pos[0])
  ydata.append(pos[1])
  zdata.append(pos[2])  
  timedata.append(time)
  speeddata.append(speed)
  
  pos[0] = pos[0] + math.cos(yaw) * dt * speed 
  pos[1] = pos[1] + math.sin(yaw) * dt * speed
  pos[2] = 0
  #print("pos", pos[0],pos[1])
  
  mpos[0] = pos[0]
  mpos[1] = pos[1]
  mpos[2] = pos[2]

  # add process noise
  yaw = yaw + np.random.randn() * noiseProcessYaw 
  #yaw = yaw + noiseProcessYaw 

  # add gyro bias drift
  #driftGyro = timestep/3600.0 * biasGyro/180.0*math.pi
  #mroll = mroll + driftGyro
  #mpitch = mpitch + driftGyro
  #myaw = myaw + driftGyro
  
  # add acc bias drift  
  '''noise = 0.5 * noiseAcc * 1e-3 * timestep**2
  mpos[0] = mpos[0] + noise
  mpos[1] = mpos[1] + noise
  mpos[2] = mpos[2] + noise'''
  # add gps noise
  mpos[0] = mpos[0] + np.random.randn() * noiseGps
  mpos[1] = mpos[1] + np.random.randn() * noiseGps
  #mpos[2] = mpos[2] + np.random.randn() * noiseGps
  #if mpos[2] < 0: mpos[2] = 0

  # steering control (path tracking)
  lateralError = distanceLineInfinite(mpos[0], mpos[1], 0, 0, gx, gy)        
  targetYaw = pointsAngle(mpos[0], mpos[1], gx, gy)      
  targetYaw = scalePIangles(targetYaw, yaw)
  angularError = distancePI(yaw, targetYaw)                         
  # apply steering control (stanley controller)
  if (doControl):
    ctl_angular = stanley_p * angularError
    #ctl_lateral = math.atan2(stanley_k * lateralError, (0.001 + abs(speed)**10))
    ctl_lateral = math.atan2(stanley_k * lateralError, (stanley_ks + abs(speed)))               
  else:
    if angularError > 0:
      ctl_angular = 0.05
    else:
      ctl_angular = -0.05
    ctl_lateral = 0


  steering = ctl_angular + ctl_lateral

  # scale by timestep
  #steering = steering * timestep

  # apply steering control with latency
  yaw = processLatency * yaw + (1.0 - processLatency) * (yaw + steering)
  
  print ('x', round(pos[0], 2), 'yaw', round(yaw/3.1415*180.0), 'steering', round(steering/3.1415*180.0), 
          'lateralError', round(lateralError, 2), 'angularError', round(angularError/3.1415*180.0, 2))

  # store values
  trueLateralError = distanceLineInfinite(pos[0], pos[1], 0, 0, gx, gy)         
  elateral.append(trueLateralError)
  #eangular.append(angularError / math.pi * 180.0)
  eangular.append(angularError)

  gxdata.append(gx * percent)
  gydata.append(0)
  gzdata.append(0)
  yawdata.append(yaw)

  mxdata.append(mpos[0])
  mydata.append(mpos[1])
  mzdata.append(mpos[2])
  
  mexdata.append(mpos[0]-pos[0])
  meydata.append(mpos[1]-pos[1])
  mezdata.append(mpos[2]-pos[2])

  ctl_angular_data.append(ctl_angular)
  ctl_lateral_data.append(ctl_lateral)
  steering_data.append(steering)

  time = time + dt    
  

minl = min(min(xdata, min(ydata, zdata)))
print('minl',minl)
maxl = max(max(xdata, max(ydata, zdata)))
print('maxl',maxl)

#zdata = 15 * np.random.random(100)
#xdata = np.sin(zdata) + 0.1 * np.random.randn(100)
#ydata = np.cos(zdata) + 0.1 * np.random.randn(100)

#fig = plt.figure()
plt.figure(figsize=(15,6))
#fig.canvas.set_window_title('RTK tracking')

'''
ax = fig.add_subplot(221, projection='3d')
ax.scatter3D(gxdata, gydata, gzdata, c='green', cmap='Greens', alpha=0.05);
ax.scatter3D(mxdata, mydata, mzdata, c='red', cmap='Greens', alpha=0.03);
ax.scatter3D(xdata, ydata, zdata, c='blue', cmap='Greens', alpha=1.0);
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(0, tracklen)
ax.set_ylim(-1.0, 1.0)
ax.set_zlim(0.0, 0.5)
ax.text2D(0.05, 0.95, "tracklen="+str(tracklen)+'m', transform=ax.transAxes)
ax.text2D(0.05, 0.90, "maxspeed="+str(maxspeed)+'m/s', transform=ax.transAxes)
ax.text2D(0.05, 0.85, "duration="+str(round(time))+'sec', transform=ax.transAxes)
#ax.text2D(0.05, 0.80, "imu="+imu, transform=ax.transAxes)
#ax.text2D(0.05, 0.75, "gyro bias drift="+str(1234)+'deg/s', transform=ax.transAxes)
ax.text2D(0.05, 0.70, "gps noise="+str(noiseGps), transform=ax.transAxes)
ax.text(25, 0, 1, "goal trajectory", color='green')
ax.text(30, 0, 1, "measured trajectory", color='red')
ax.text(35, 0, 1, "true trajectory", color='blue')


ax = fig.add_subplot(222)
ax.plot(timedata,mexdata,'-r',label='merror_x',alpha=0.8)        
ax.plot(timedata,meydata,'-g',label='merror_y',alpha=0.8)        
ax.plot(timedata,mezdata,'-b',label='merror_z',alpha=0.8)        
ax.legend()
ax.grid()
ax.set_xlabel('time [s]')
ax.set_ylabel('measurement error [m]')

ax = fig.add_subplot(223)
ax.plot(timedata,elateral,'-r',label='lateral_err',alpha=0.8)        
ax.plot(timedata,eangular,'-g',label='angular_err',alpha=0.8)
ax.plot(timedata,speeddata,'-b',label='speed',alpha=0.8)
ax.legend()
ax.grid()
ax.set_xlabel('time [s]')
ax.set_ylabel('control error [m]')

ax = fig.add_subplot(224)
ax.plot(timedata,ctl_angular_data,'-r',label='angular_ctl',alpha=0.8)        
ax.plot(timedata,ctl_lateral_data,'-b',label='lateral_ctl',alpha=0.8)
ax.plot(timedata,steering_data,'-g',label='steering',alpha=0.8)        
#ax.set_ylim(-0.5, 0.5)
ax.legend()
ax.grid()
ax.set_xlabel('time [s]')
ax.set_ylabel('control output')
'''

plt.plot(timedata, speeddata, '-b',label='speed',alpha=0.8);
plt.plot(timedata, elateral, '-r',label='lateral_err',alpha=0.8);
plt.plot(timedata, yawdata, '-g',label='yaw',alpha=0.8);
plt.plot(timedata, steering_data, '-y',label='steering',alpha=0.8);


plt.legend()
plt.grid()

plt.tight_layout()
plt.show()              


