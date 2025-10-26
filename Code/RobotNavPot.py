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
pot = Potential.Potential(difficulty=3, random=True)


# position control loop: gain and timer
kpPos = 1.0
Vmax = 4.0  # maximum velocity in m/s
positionCtrlPeriod = 0.2#0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
kpOrient = 2.5
orientationCtrlPeriod = 0.02#0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)


# list of way points list of [x coord, y coord]
WPlist = [ [x0,y0], [0, 0]]
for i in range(9):
    radius = 5.0
    angle = -i * (2* math.pi/8)
    WPlist.append([radius * math.cos(angle), radius * math.sin(angle)])
WPlist.append([0, 0])

#threshold for change to next WP
epsilonWP = 1.8
# init WPManager
WPManager = rob.WPManager(WPlist, epsilonWP)


# duration of scenario and time step for numerical integration
t0 = 0.0
tf = 500.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)


def getSourceDirection():
    range = 4
    maxPotential = max(sampleStorage.keys())
    positions = sampleStorage[maxPotential]
    avgX = sum([pos[0] for pos in positions]) / len(positions)
    avgY = sum([pos[1] for pos in positions]) / len(positions)
    return (range*avgX, range*avgY)

def storingSamples():
    roundedCoordinates = [round(robot.x), round(robot.y)]
    roundedPotential = int(pot.value([robot.x, robot.y]) // 10)
    if roundedPotential in sampleStorage:
        if roundedCoordinates not in sampleStorage[roundedPotential]:
            sampleStorage[roundedPotential].append(roundedCoordinates)
    else:
        sampleStorage[roundedPotential] = [roundedCoordinates]
    # print(f"Echantillonnage en {roundedCoordinates} avec un potentiel de {roundedPotential}")

def computeSourceCenter():
    sumX = 0.0
    sumY = 0.0
    count = 0
    for positions in sampleStorage.values():
        for pos in positions:
            sumX += pos[0]
            sumY += pos[1]
            count += 1
    if count > 0:
        return (sumX / count, sumY / count)
    else:
        return (0.0, 0.0)

#firstIter = True


possibleStates = ["gotocenter",
                   "circle_sampling",
                   "gotosource",
                   "outlining_source",
                   "returning_home"]

currentState = "gotocenter"

# storage for sampled potentials
# key: potential value //10, value: list of positions [x,y]
sampleStorage = {}

# arbitrary pollution threshold for source approach
pollutionThreshold = 290

# variable to check if robot is getting closer to the source
previousPot = 0.0

# initialize control inputs
Vr = 0.0
thetar = 0.0
omegar = 0.0

# loop on simulation time
for t in simu.t: 


    # check if pollution threshold is reached to stop near source
    if pot.value([robot.x, robot.y]) >= pollutionThreshold-10 \
            and currentState not in ["outlining_source", "returning_home"]:
        currentState = "outlining_source"
        sampleStorage = {}
        WPManager.WPList = []
        WPManager.currentWP = None
        WPManager.xr = robot.x
        WPManager.yr = robot.y
        print("Source atteinte, contournement de la source")
        
        

    # storing samples when in circle_sampling state
    if currentState == "circle_sampling":
        storingSamples()


    # WP navigation: switching condition to next WP of the list
    if (WPManager.distanceToCurrentWP(robot.x, robot.y) < epsilonWP):

        if WPManager.xr==0.0 and WPManager.yr==0.0:
            if currentState=="gotocenter":
                currentState="circle_sampling"
                print("Arrivé au centre, début du cercle de sampling")
            
            elif currentState=="circle_sampling":
                currentState="gotosource"
                WPManager.WPList.append(getSourceDirection())
                print("Sampling terminé, direction la source !")
        

        # outlining the source
        if currentState == "outlining_source":
            storingSamples()

            angle = math.pi/8
            # angle = math.pi/4

            WPrange = 2.0

            if pot.value([robot.x, robot.y]) < previousPot < pollutionThreshold:
                nextX = robot.x + WPrange*math.cos(robot.theta - angle)
                nextY = robot.y + WPrange*math.sin(robot.theta - angle)

            elif pot.value([robot.x, robot.y]) > previousPot > pollutionThreshold:
                nextX = robot.x + WPrange*math.cos(robot.theta + angle)
                nextY = robot.y + WPrange*math.sin(robot.theta + angle)
            else:
                nextX = robot.x + WPrange*math.cos(robot.theta)
                nextY = robot.y + WPrange*math.sin(robot.theta)

            # checking if the robot has already been at these coordinates (meaning the contour is over)
            roundedPotential = int(pot.value([robot.x, robot.y]) // 10)
            roundedNextCoordinates = [round(nextX), round(nextY)]
            if roundedPotential in sampleStorage and roundedNextCoordinates in sampleStorage[roundedPotential]:
                currentState = "returning_home"
                WPManager.WPList = []
                WPManager.WPList.append([x0, y0])
                print("Contour terminé, retour à la base")

                
            else:
                WPManager.WPList.append([nextX, nextY])


        previousPot = pot.value([robot.x, robot.y])

        if not WPManager.isWPListEmpty():
            WPManager.switchToNextWP()


    # position control loop
    if timerPositionCtrl.isEllapsed(t):

        # Calculate distance to current waypoint
        distance = WPManager.distanceToCurrentWP(robot.x, robot.y)
        
        # Calculate desired linear velocity (proportional control)
        #Kv = 1  # gain for linear velocity control
        Vr = kpPos * distance
        
        # Limit maximum velocity if needed
        if Vr > Vmax:
            Vr = Vmax

        # reference orientation
        thetar = math.atan2(WPManager.yr - robot.y, WPManager.xr - robot.x)
        
        if math.fabs(robot.theta-thetar)>math.pi:
            thetar = thetar + math.copysign(2*math.pi,robot.theta)

    # orientation control loop
    if timerOrientationCtrl.isEllapsed(t):
        # angular velocity control input
        orientation_error = thetar - robot.theta
        omegar = kpOrient * orientation_error
    

    # assign control inputs to robot
    robot.setV(Vr)
    robot.setOmega(omegar)    
    
    # integrate motion
    robot.integrateMotion(dt)

    # store data to be plotted   
    simu.addData(robot, WPManager, Vr, thetar, omegar, pot.value([robot.x,robot.y]))
# end of loop on simulation time

# print(sampleStorage)




# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

#simu.plotXYTheta(2)
#simu.plotVOmega(3)

#simu.plotPotential(4)



#simu.plotPotential3D(5)


# show plots
# plt.show()





# # Animation *********************************
fig = plt.figure(1)
#ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-25, 25), ylim=(-25, 25))


robotBody, = ax.plot([], [], 'o-', lw=2)
robotDirection, = ax.plot([], [], '-', lw=2, color='k')
wayPoint, = ax.plot([], [], 'o-', lw=2, color='b')
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
potential_template = 'potential = %.1f'
potential_text = ax.text(0.05, 0.1, '', transform=ax.transAxes)
WPArea, = ax.plot([], [], ':', lw=1, color='b')
sourceX, sourceY = computeSourceCenter()
sourcePoint, = ax.plot(sourceX, sourceY, marker='*', markersize=14, markeredgecolor='k')

thetaWPArea = np.arange(0.0,2.0*math.pi+2*math.pi/30.0, 2.0*math.pi/30.0)
xWPArea = WPManager.epsilonWP*np.cos(thetaWPArea)
yWPArea = WPManager.epsilonWP*np.sin(thetaWPArea)

def initAnimation():
    robotDirection.set_data([], [])
    robotBody.set_data([], [])
    wayPoint.set_data([], [])
    WPArea.set_data([], [])
    robotBody.set_color('g')
    robotBody.set_markersize(10)    
    time_text.set_text('')
    potential_text.set_text('')
    sourcePoint.set_data([], [])
    return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea, sourcePoint

def animate(i):  
    robotBody.set_data([simu.x[i]], [simu.y[i]])          
    wayPoint.set_data([simu.xr[i]], [simu.yr[i]])
    WPArea.set_data([simu.xr[i]+xWPArea.transpose()], [simu.yr[i]+yWPArea.transpose()])    
    thisx = [simu.x[i], simu.x[i] + 1*math.cos(simu.theta[i])]
    thisy = [simu.y[i], simu.y[i] + 1*math.sin(simu.theta[i])]
    robotDirection.set_data(thisx, thisy)
    time_text.set_text(time_template%(i*simu.dt))
    potential_text.set_text(potential_template%(pot.value([simu.x[i],simu.y[i]])))
    sourcePoint.set_data([sourceX], [sourceY])
    return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea, sourcePoint

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(simu.t), 5),
    interval=25, blit=True, init_func=initAnimation, repeat=False)
#interval=25

#ani.save('robot.mp4', fps=15)

plt.show()

# Save GIF
#ani.save('SourceFinder.gif', writer='pillow', fps=20, dpi=80)
