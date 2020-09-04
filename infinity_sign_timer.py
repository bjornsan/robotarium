import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import time
import numpy as np

N = 1

initial_conditions = np.array(np.mat('0; 0; 1.2'))

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=True)

xVec = np.array([-2, -1, 0, 1, 2])
yVec = np.array([0, 0, 0, 0, 0])
r.axes.plot(xVec, yVec, color='red')

xVec2 = np.array([0, 0, 0, 0, 0])
yVec2 = np.array([-2, -1, 0, 1, 2])
r.axes.plot(xVec2, yVec2, color='green')


xVec3 = np.array([-3, -1, 0, 1, 3])
yVec3 = np.array([-2, -1, 0, 1, 2])
r.axes.plot(xVec3, yVec3, color='blue')

xVec4 = np.array([-3, -1, 0, 1, 3])
yVec4 = np.array([2, 1, 0, -1, -2])
r.axes.plot(xVec4, yVec4, color='pink')

laneRadius = float(0.2)
line_width = 5

circle = plt.Circle((0.2,0),  laneRadius, color='r',fill = False, linewidth = line_width)
circle2 = plt.Circle((-0.2, 0), laneRadius, color="r", fill= False, linewidth = line_width)
r.axes.add_artist(circle)
r.axes.add_artist(circle2)

vel = np.array([0.1, 0])
vel.shape = (2,1)
turnLeft = np.array([0.12, 1.2])
turnLeft.shape = (2,1)
turnRight = np.array([0.12, -1.2])
turnRight.shape = (2,1)

xyz = r.get_poses()
r.step()

#flag 0 = turn right, flag 1 = drive straight, flag 2 = turn left
initial_i = 0
flag = 0
for i in range(600):
  print(i)

  if i < 150:
    r.set_velocities(0, turnLeft)

  if i - initial_i == 150:
    initial_i = i
    if flag == 0:
      flag = 1
      r.set_velocities(0, turnRight)
    elif flag == 1:
      flag = 0
      r.set_velocities(0, turnLeft)
  

  xyz = r.get_poses()
  r.step()


r.call_at_scripts_end()

