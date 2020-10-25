from geometry import compute_heading_error, quaternion_for
from controller import control

from xpc import XPlaneConnect
import ilqgamespy

import math
import time

import sys


runwayOriginX = -25163.9477
runwayOriginY = 33693.3450
runwayEndX = -22742.57617
runwayEndY = 31956.02344
runwayHeading = 54.331

timeStep = 0.1
    

# setup ilqgames solver parameters
params = ilqgamespy.SolverParams()

params.max_backtracking_steps = 100
params.linesearch = True
params.enforce_constraints_in_linesearch = False
params.trust_region_size = 0.5
params.initial_alpha_scaling = 0.1
params.convergence_tolerance = 0.1
params.state_regularization = 0.0
params.control_regularization = 0.0
params.open_loop = False

problem = ilqgamespy.PyProblem(params)

# create XPC object
xp_client = XPlaneConnect()

# setup properties

groundspeed_dref = "sim/flightmodel/position/groundspeed"           # m/s
heading_dref = "sim/flightmodel/position/psi"                       # in degrees
position_dref = ["sim/flightmodel/position/local_x",
                 "sim/flightmodel/position/local_y",
                 "sim/flightmodel/position/local_z"]
throttle_dref = "sim/flightmodel/engine/ENGN_thro"

#orientation_dref = ["sim/flightmodel/position/theta",
#                    "sim/flightmodel/position/phi",
#                    "sim/flightmodel/position/q"]


## orientation test
#theta, phi, q = xp_client.getDREFs(orientation_dref)
#theta, phi = math.radians(theta[0]), math.radians(phi[0])
#quaternion = quaternion_for(theta, phi, math.radians(runwayHeading + 180))
#xp_client.sendDREF("sim/flightmodel/position/q", quaternion)

# release park brake
xp_client.sendDREF("sim/flightmodel/controls/parkbrake", 0)
time.sleep(0.1)

state = 0

for i in range(1000):

    gs, psi, throttle = xp_client.getDREFs([groundspeed_dref, heading_dref, throttle_dref])
    gs, psi, throttle = gs[0], psi[0], throttle[0]
    
    # resolve every 5 seconds
    if i % 50 == 0:
        x, _, z = xp_client.getDREFs(position_dref)
        update_states = [0] * problem.dimension
        update_states[ilqgamespy.kPxIdx], update_states[ilqgamespy.kPyIdx] = x[0], z[0]
        update_states[ilqgamespy.kThetaIdx] = psi - runwayHeading
        update_states[ilqgamespy.kVIdx] = gs
        op = ilqgamespy.solver(problem, update_states).FinalOperatingPoint()
        headings, velocities = ilqgamespy.getStates(op)
        state = 0
    
    # send controls to xplane
    heading, velocity = headings[state], velocities[state]
    
    print(runwayHeading + heading, velocity)

    heading_err = compute_heading_error(runwayHeading + heading, psi)
    control(xp_client, velocity, gs, throttle, heading_err)
    state += 1
    
    time.sleep(timeStep)
        
        
    
 
    
    
    
