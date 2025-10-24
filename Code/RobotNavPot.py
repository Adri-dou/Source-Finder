# -*- coding: utf-8 -*-
"""
Way Point navigtion

(c) S. Bertrand
"""

import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential

# robot
x0 = -20.0
y0 = -20.0
theta0 = np.pi/4.0
robot = rob.Robot(x0, y0, theta0)


# potential
pot = Potential.Potential(difficulty=1, random=False)


# position control loop: gain and timer
kpPos = 0.8
positionCtrlPeriod = 0.2#0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
kpOrient = 2.5
orientationCtrlPeriod = 0.05#0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)



# list of way points list of [x coord, y coord]
WPlist = [ [x0,y0], [0, 0], [10,10], [-10,10], [-10,-10], [10,-10], [0,0] ]
#threshold for change to next WP
epsilonWP = 0.2
# init WPManager
WPManager = rob.WPManager(WPlist, epsilonWP)


# duration of scenario and time step for numerical integration
t0 = 0.0
tf = 200.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)


# initialize control inputs
Vr = 0.0
thetar = 0.0
omegar = 0.0

firstIter = True



# loop on simulation time
for t in simu.t: 
   
   
    # WP navigation: switching condition to next WP of the list
    if (not WPManager.isWPListEmpty()) and (WPManager.distanceToCurrentWP(robot.x, robot.y) < epsilonWP):
        WPManager.switchToNextWP()


    # position control loop
    if timerPositionCtrl.isEllapsed(t):
        # Calculate distance to current waypoint
        distance = WPManager.distanceToCurrentWP(robot.x, robot.y)
        
        # Calculate desired linear velocity (proportional control)
        Kv = 1  # gain for linear velocity control
        Vr = Kv * distance
        
        # Limit maximum velocity if needed
        Vmax = 2.0  # maximum velocity in m/s
        if Vr > Vmax:
            Vr = Vmax

        # reference orientation
        thetar = math.atan2(WPManager.yr - robot.y, WPManager.xr - robot.x)
        
        if math.fabs(robot.theta-thetar)>math.pi:
            thetar = thetar + math.copysign(2*math.pi,robot.theta)

    # orientation control loop
    if timerOrientationCtrl.isEllapsed(t):
        # angular velocity control input
        Kw = 2 # gain for angular velocity control
        orientation_error = thetar - robot.theta
        omegar = Kw * orientation_error
    
    
    # assign control inputs to robot
    robot.setV(Vr)
    robot.setOmega(omegar)    
    
    # integrate motion
    robot.integrateMotion(dt)

    # store data to be plotted   
    simu.addData(robot, WPManager, Vr, thetar, omegar, pot.value([robot.x,robot.y]))
    
    
# end of loop on simulation time


# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

simu.plotXYTheta(2)
#simu.plotVOmega(3)

simu.plotPotential(4)



simu.plotPotential3D(5)


# show plots






# # Animation *********************************
fig = plt.figure(1)
#ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-25, 25), ylim=(-25, 25))
ax.grid()
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')

robotBody, = ax.plot([], [], 'o-', lw=2)
robotDirection, = ax.plot([], [], '-', lw=1, color='k')
wayPoint, = ax.plot([], [], 'o-', lw=2, color='b')
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
potential_template = 'potential = %.1f'
potential_text = ax.text(0.05, 0.1, '', transform=ax.transAxes)
WPArea, = ax.plot([], [], ':', lw=1, color='b')

thetaWPArea = np.arange(0.0,2.0*math.pi+2*math.pi/30.0, 2.0*math.pi/30.0)
xWPArea = WPManager.epsilonWP*np.cos(thetaWPArea)
yWPArea = WPManager.epsilonWP*np.sin(thetaWPArea)

def initAnimation():
    robotDirection.set_data([], [])
    robotBody.set_data([], [])
    wayPoint.set_data([], [])
    WPArea.set_data([], [])
    robotBody.set_color('r')
    robotBody.set_markersize(10)    
    time_text.set_text('')
    potential_text.set_text('')
    return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea  

def animate(i):  
    robotBody.set_data([simu.x[i]], [simu.y[i]])          
    wayPoint.set_data([simu.xr[i]], [simu.yr[i]])
    WPArea.set_data([simu.xr[i]+xWPArea.transpose()], [simu.yr[i]+yWPArea.transpose()])    
    thisx = [simu.x[i], simu.x[i] + 0.5*math.cos(simu.theta[i])]
    thisy = [simu.y[i], simu.y[i] + 0.5*math.sin(simu.theta[i])]
    robotDirection.set_data(thisx, thisy)
    time_text.set_text(time_template%(i*simu.dt))
    potential_text.set_text(potential_template%(pot.value([simu.x[i],simu.y[i]])))
    return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(simu.t)),
    interval=4, blit=True, init_func=initAnimation, repeat=False)
#interval=25

#ani.save('robot.mp4', fps=15)

plt.show()