import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.misc import *
from rps.utilities.barrier_certificates import *
from rps.utilities.controllers import *

import time
import numpy as np

N = 1
initial_conditions = np.array(np.mat('0.01; 0.01; 0'))
r = robotarium.Robotarium(number_of_robots = N, show_figure = True, initial_conditions = initial_conditions, sim_in_real_time = True)

x = r.get_poses()
r.step()


velForLeft = np.array([0.6, 1.57])
velForLeft.shape = (2,1)
velForRight = np.array([0.6,-1.57])
velForRight.shape = (2,1)
vel = np.array([0.0, 0.0])
vel.shape = (2,1)
piOverTwo = np.pi/float(2)


flag = 0
for i in range(1000):
 
  posVec = r.get_poses()
 
  x = posVec.item(0)
  y = posVec.item(1)
  psi = posVec.item(2)
  test = np.sin(psi-piOverTwo)
  print(test)

  if flag == 0:
    if np.sin(psi-piOverTwo) <= 0:
      flag = 1
      print("change to ONE")
  elif flag == 1:
    if np.sin(psi-piOverTwo) >= 0:
      flag = 2
      print("change to TWO")
  elif flag == 2:
    if np.sin(psi-piOverTwo) >= 0:
      flag = 3
      print("change to THREE")
  elif flag == 3:
    if np.sin(psi-piOverTwo) <= 0:
      flag = 0
      print("Change back to ZERO")

  if flag == 0 or flag == 1:
    vel = velForLeft
  elif flag == 2 or flag == 3:
    vel = velForRight

  r.set_velocities(0, vel)

  r.step()
	
r.call_at_scripts_end()
