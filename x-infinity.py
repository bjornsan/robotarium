###						###
#						  #
#	x-infinity.py - Björn Andersson	 	  #
#		Autum - 2020			  #
###						###

import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time

# Instantiate Robotarium object
N = 1
initial_conditions = np.array(np.mat('0.01;-0.01;0'))
#initial_conditions = np.array(np.mat('1 0.5 -0.5 0 0.28; 0.8 -0.3 -0.75 0.1 0.34; 0 0 0 0 0'))
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=True)

arrayExtra = np.array([2.5, 0, 0])
arrayExtra.shape = (3,1)
goal_points = arrayExtra

#--- Create single integrator pose controller
position_controller = create_si_position_controller();
#--- Create converter that takes single integrator
#    velocity vector gives the equivalent unicycle velocity vector
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

# get current position and pose so that the graphics lines coming below
#  can draw the initial position and pose
x = r.get_poses()

# Plotting Parameters
CM = np.random.rand(N,3) # Random Colors
goal_marker_size_m = 0.2
robot_marker_size_m = 0.15
marker_size_goal = determine_marker_size(r,goal_marker_size_m)
marker_size_robot = determine_marker_size(r, robot_marker_size_m)
font_size = determine_font_size(r,0.1)
line_width = 5

# Create Goal Point Markers
#Text with goal identification
goal_caption = ['G{0}'.format(ii) for ii in range(goal_points.shape[1])]
#Arrow for desired orientation
goal_orientation_arrows = [r.axes.arrow(goal_points[0,ii], goal_points[1,ii], goal_marker_size_m*np.cos(goal_points[2,ii]), goal_marker_size_m*np.sin(goal_points[2,ii]), width = 0.02, length_includes_head=True, color = CM[ii,:], zorder=-2)
for ii in range(goal_points.shape[1])]
#Plot text for caption
goal_points_text = [r.axes.text(goal_points[0,ii], goal_points[1,ii], goal_caption[ii], fontsize=font_size, color='k',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=-3)
for ii in range(goal_points.shape[1])]
goal_markers = [r.axes.scatter(goal_points[0,ii], goal_points[1,ii], s=marker_size_goal, marker='s', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width,zorder=-3)
for ii in range(goal_points.shape[1])]
robot_markers = [r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width) 
for ii in range(goal_points.shape[1])]


# ----  number of simulation steps
countMax        = 1000       

# ----  draw infinity track using four  vectors
#
#    #  .. .... .... .... .... ..  #		   
#    # pointTwo  	pointFour  #
#    #             X  		   #
#    # pointThree	pointOne   #
#    #  .. .... .... .... .... ..  #               
#
pointOne = [0.8, -0.4]
pointTwo = [-0.8, 0.4]
pointThree = [-0.8, -0.4]
pointFour = [0.8, 0.4]

xValuesVectorOne = [pointOne[0], pointTwo[0]]
yValuesVectorOne = [pointOne[1], pointTwo[1]]

xValuesVectorTwo = [pointThree[0], pointFour[0]]
yValuesVectorTwo = [pointThree[1], pointFour[1]]

xValuesVectorThree = [pointTwo[0], pointThree[0]]
yValuesVectorThree = [pointTwo[1], pointThree[1]]

xValuesVectorFour = [pointFour[0], pointOne[0]]
yValuesVectorFour = [pointFour[1], pointOne[1]]

r.axes.plot(xValuesVectorOne, yValuesVectorOne, color='green' )
r.axes.plot(xValuesVectorTwo, yValuesVectorTwo, color='green')

r.axes.plot(xValuesVectorThree, yValuesVectorThree, color='blue', linestyle='--') 
r.axes.plot(xValuesVectorFour, yValuesVectorFour, color='blue', linestyle='--')



# ----  parameters of the control law
maxLinearSpeed  = float(0.2) # extracted from line 47 of robotarium_abc.py
maxAngularSpeed = float(3.9) # extracted from line 48 of robotarium_abc.py
linearSpeed     = float(0.8)
if ( linearSpeed * linearSpeed > maxLinearSpeed * maxLinearSpeed ):
    linearSpeed = maxLinearSpeed

num_waypoints = 4

 
waypoints = np.zeros((2,num_waypoints), dtype=np.float64 )

# [0] = x, [1] = y

waypoints[0][0] = pointTwo[0]
waypoints[0][1] = pointThree[0]
waypoints[0][2] = pointFour[0]
waypoints[0][3] = pointOne[0]
waypoints[1][0] = pointTwo[1]
waypoints[1][1] = pointThree[1]
waypoints[1][2] = pointFour[1]
waypoints[1][3] = pointOne[1]



waypointIndexNow = 0

#   And now specify how close you have to come to
#   a waypoint before you abandon it and target the next one
distance_tolerance = 0.01


#
# check for closest waypoint from startpoint
#

x_pos = x.item(0)
y_pos = x.item(1)

closest_x = float(2)
closest_y = float(2)


indexOnStartup = 0

# looping through all waypoints, comparing against the robots startup position.

for i in range(len(waypoints)):

  for ii in range(len(waypoints[i])):
   
    if (abs(x_pos - waypoints[0][ii]) < abs(x_pos - closest_x)):
      if x_pos > 0 and waypoints[0][ii] > 0: 
        closest_x = waypoints[0][ii]
      elif x_pos < 0 and waypoints[0][ii] < 0:
        closest_x = waypoints[0][ii]
    
    if (abs(y_pos - waypoints[1][ii]) < abs(y_pos-closest_y)):
      if y_pos > 0 and waypoints[1][ii] > 0:
        closest_y = waypoints[1][ii]
      elif y_pos < 0 and waypoints[1][ii] < 0:
        closest_y = waypoints[1][ii]
      
     
# Finding correct index for startup. For some reason didnt work with the previous loop. 
# If the robot is within 5 centimeters from origo it will choose the first waypoint. 
# Otherwise it will decide which is the closest one.  
for i in range(len(waypoints)):
  for ii in range(len(waypoints[i])):
    if x_pos >( x_pos + 0.05) and y_pos > (y_pos + 0.05):
      if waypoints[0][ii] == closest_x and waypoints[1][ii] == closest_y:
        indexOnStartup = ii


waypointIndexNow =indexOnStartup
closestWaypointOnStartup = np.array([waypoints[0][indexOnStartup], waypoints[1][indexOnStartup]])
waypointNow = closestWaypointOnStartup
waypointNow.shape = (2,1)



#
#  initialize the simulation
r.step()
count = 0
while ( (count < countMax)):

    # Get poses of agents
    x = r.get_poses()

    # -------------- retain this from here
    #Update Plot
    # Update Robot Marker Plotted Visualization
    for i in range(x.shape[1]):
        robot_markers[i].set_offsets(x[:2,i].T)
        # This updates the marker sizes if the figure window size is changed. 
        # This should be removed when submitting to the Robotarium.
#        robot_markers[i].set_sizes([determine_marker_size(r, robot_marker_size_m)])

    #  is needed for plotting
    for j in range(goal_points.shape[1]):
        goal_markers[j].set_sizes([determine_marker_size(r, goal_marker_size_m)])


    # Create safe control inputs (i.e., no collisions)
    #dxu = uni_barrier_cert(dxu, x)
    # -------------- up to here

    # increment simulation step count
    count = count + 1

    # calculate present location and orientation
    xRob     = float(x.item(0))
    yRob     = float(x.item(1))
    psiRob   = float(x.item(2))
    # extract coordinates of current waypoint
    wayPtX = waypointNow[0]
    wayPtY = waypointNow[1]
    
    # if we have come close to the current waypoint, then
    #      target the next one
    distanceSq = np.square( xRob - wayPtX ) + np.square( yRob - wayPtY ) 
    if ( distanceSq <= distance_tolerance  ):
        waypointIndexNow = waypointIndexNow + 1
        #  if we have reached the last column of the waypoints array
        #     then go back to the zeroth column
        if   ( waypointIndexNow >= num_waypoints):
            waypointIndexNow = 0
        waypointNow = waypoints[:, waypointIndexNow ]
        waypointNow.shape = (2,1)

    # for tw-element column vector of currenr x and y coordinated of robot
    pos = np.array( [ xRob , yRob ] )
    pos.shape = (2,1)

    # get the v_x and v_y (rectilinear) velocity vector
    #   from the rps function, by supplying the
    #   current robot x, y coordinates and those of the waypoint
    
    vel = position_controller( pos, waypointNow)
    #  conver the rectilinear velocity vector into
    #   the tangential velocity and turning rate
    #   needed for steering as a unicycle
    uni_vel = si_to_uni_dyn ( vel , x )
    # Set the velocity command
    r.set_velocities( 0, uni_vel)

    # Iterate the simulation
    r.step()

#Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
