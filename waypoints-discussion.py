import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time

# Instantiate Robotarium object
N = 1
initial_conditions = np.array(np.mat('0.01;0.01;0'))
#initial_conditions = np.array(np.mat('1 0.5 -0.5 0 0.28; 0.8 -0.3 -0.75 0.1 0.34; 0 0 0 0 0'))
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=True)

# Define goal points by removing orientation from poses - modified to lie outside the shown area
arrayExtra = np.array([2.5, 0, 0])
arrayExtra.shape = (3,1)
goal_points = arrayExtra
#goal_points = generate_initial_conditions(N, width=r.boundaries[2]-2*r.robot_diameter, height = r.boundaries[3]-2*r.robot_diameter, spacing=0.5)

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


# ----  choose lane radius and the number of simulation steps
laneRadius      = float(0.4)
countMax        = 2000           # integer number of steps

# ----  draw infinity track using two circular track 
circle = plt.Circle( (-laneRadius,0),  laneRadius, color='r',fill = False, linewidth = line_width)
circle2 = plt.Circle( (laneRadius,0),  laneRadius, color='r',fill = False, linewidth = line_width)
r.axes.add_artist(circle)
r.axes.add_artist(circle2)

# ----  parameters of the control law
maxLinearSpeed  = float(0.2) # extracted from line 47 of robotarium_abc.py
maxAngularSpeed = float(3.9) # extracted from line 48 of robotarium_abc.py
linearSpeed     = float(0.08)
if ( linearSpeed * linearSpeed > maxLinearSpeed * maxLinearSpeed ):
    linearSpeed = maxLinearSpeed

#------  define waypoints in two rounds
#  having a small number of waypoints makes the robot
#   move fast, but not exactly on the track
#  having many waypoints makes the robot move relatively slowly
#   but the robot does stick more closely to the track
num_waypoints = 20;

# initialize the_vec as an array of zeros (  64 bit floating point numbers)
th_vec = np.zeros(num_waypoints,dtype = np.float64 )
# make the elements of th_vec equally spaced from 0 to 2 pi radians
for i in range(num_waypoints):
    th_vec[i] = 2 * np.pi * i / num_waypoints

# initialize the array waypoints as an array where
#   each column has two elements 
#   note that the number of columns is two times num_waypoints
waypoints = np.zeros((2,2*num_waypoints),dtype = np.float64 )

#-- round 1, define waypoints on the left circle
for i in range(num_waypoints) :
    theta = th_vec[i]
    waypoints[0,i] = -1*laneRadius + laneRadius * np.cos(theta)
    waypoints[1,i] = laneRadius * np.sin(theta)

#-- round 2, define waypoints on the right circle
for i in range(num_waypoints) :
    theta = th_vec[i]
    waypoints[0,i + num_waypoints] = laneRadius  -  laneRadius * np.cos(theta) 
    waypoints[1,i + num_waypoints] = laneRadius * np.sin(theta)

#  specify the fist waypoint 
#    PERHAPS in your code, you could find out which of the waypoints
#    is closest to the initial condition, and then
#    set waypointNow and waypointIndexNow accordingly
#      you can find the initial condition by calling r.get_poses()
#  
#  extract column number 0 from the array called waypoints
waypointNow = waypoints[:,0]
waypointIndexNow = 0
#   And now to really tell the Python interpreter that what is
#       is stored on waypointNow is a column vector with two elements
waypointNow.shape = (2,1)
#   And now specify how close you have to come to
#   a waypoint before you abandon it and target the next one
distance_tolerance = 0.01

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
        if   ( waypointIndexNow >=  2 * num_waypoints):
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

print(count)

#Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
