
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


class OrbitalScanner:
    def __init__(self, robot, pot, step, rad, orbSubDiv, dirVec):
        self.step = step
        self.rad = rad
        self.pot = pot
        self.dirVec = dirVec
        self.robot = robot
        self.orbSubDiv = orbSubDiv
        self.nextWp = [robot.x + dirVec[0]*step, robot.y + dirVec[1]*step]
        self.state = 0
        self.highestPot = [0, 0, 0]  # [Pot.value, posX, posY]
        self.orbitHp = [0, 0, 0]
        self.radialCenterHP = 0
        self.radialCenter = [0, 0]
        self.orbitWP = []
        self.orbitWPId = 0

        self.debugFlag = True

    def update(self):
        if self.state == 0:
            if(self.highestPot[0] < self.pot.value([self.robot.x, self.robot.y])):
                self.highestPot = [self.pot.value(
                    [self.robot.x, self.robot.y]), self.robot.x, self.robot.y]
                self.radialCenter = [self.robot.x, self.robot.y]
                self.radialCenterHP = self.pot.value(
                    [self.robot.x, self.robot.y])
                self.nextWp = [self.robot.x + self.dirVec[0]
                               * self.step, self.robot.y + self.dirVec[1]*self.step]

            else:
                if self.debugFlag:
                    self.state = 1
                # self.debugFlag = False
        elif self.state == 1:
            self.computeOrbitWP()
            self.state = 2
            self.highestPot = [0, self.robot.x, self.robot.y]
            self.orbitWPId = 0

        elif self.state == 2:

            self.orbitWPId += 1
            self.nextWp = self.orbitWP[self.orbitWPId]

            if(self.highestPot[0] <= self.pot.value([self.robot.x, self.robot.y])):
                self.highestPot = [self.pot.value(
                    [self.robot.x, self.robot.y]), self.robot.x, self.robot.y]
            else:
                if(self.highestPot[0] >= self.radialCenterHP):
                    self.highestPot = [0, self.robot.x, self.robot.y]
                    self.state = 0
                    self.nextWp = self.radialCenter
                    self.computeNewVecDir()

            if(self.orbitWPId >= self.orbSubDiv-1):
                self.orbitWPId = self.orbSubDiv-2
                print("epi center found", self.radialCenter, self.radialCenterHP)
                self.state = 3
                self.nextWp = self.radialCenter

            self.orbitHp = self.highestPot

    def computeOrbitWP(self):
        self.orbitWP = []
        for i in range(self.orbSubDiv):
            wpX = math.cos(((math.pi*2)/self.orbSubDiv)*i) * \
                self.rad + self.radialCenter[0]
            wpY = math.sin(((math.pi*2)/self.orbSubDiv)*i) * \
                self.rad + self.radialCenter[1]
            self.orbitWP.append([wpX, wpY])

    def computeNewVecDir(self):
        self.dirVec[0] = self.orbitHp[1]-self.radialCenter[0]
        self.dirVec[1] = self.orbitHp[2]-self.radialCenter[1]
        d = math.sqrt(math.pow(self.dirVec[0], 2)+math.pow(self.dirVec[1], 2))
        self.dirVec[0] /= d
        self.dirVec[1] /= d



# robot
x0 = -20.0
y0 = -20.0
theta0 = np.pi/4.0
robot = rob.Robot(x0, y0, theta0)


# potential

pot = Potential.Potential(difficulty=1, random=True)


positionCtrlPeriod = 0.2  # 0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
orientationCtrlPeriod = 0.05  # 0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)


t0 = 0.0
tf = 500.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)

Vr = 0.0
thetar = 0.0
omegar = 0.0

k1 = 4
k2 = 1


dirVec = [1, 0]
orbitalScanner = OrbitalScanner(robot, pot, 5, 7, 20, dirVec)

epsilonWP = 0.1
WPManager = rob.WPManager([orbitalScanner.nextWp], epsilonWP)


# loop on simulation time
for t in simu.t:
    # WP navigation: switching condition to next WP of the list

    d = WPManager.distanceToCurrentWP(robot.x, robot.y)

    if(d < WPManager.epsilonWP):
        orbitalScanner.update()
        WPManager.WPList.append(orbitalScanner.nextWp)
        WPManager.switchToNextWP()

    if timerPositionCtrl.isEllapsed(t):
        # A COMPLETER EN TD : calcul de Vr
        Vr = k2*math.sqrt(math.pow(WPManager.xr-robot.x, 2) +
                          math.pow(WPManager.yr-robot.y, 2))

        # A COMPLETER EN TD : calcul de thetar
        thetar = np.arctan2((WPManager.yr-robot.y), (WPManager.xr-robot.x))

        # !!!!!
        # A COMPRENDRE EN TD : quelle est l'utilité des deux lignes de code suivantes ?
        #     (à conserver après le calcul de thetar)
        if math.fabs(robot.theta-thetar) > math.pi:
            thetar = thetar + math.copysign(2*math.pi, robot.theta)
        # !!!!!

    # orientation control loop
    if timerOrientationCtrl.isEllapsed(t):
        # angular velocity control input
        # !!!!!
        # A COMPLETER EN TD : calcul de omegar
        omegar = k1*(thetar-robot.theta)
        # !!!!!

    # apply control inputs to robot
    robot.setV(Vr)
    robot.setOmega(omegar)

    # integrate motion
    robot.integrateMotion(dt)

    # store data to be plotted
    simu.addData(robot, WPManager, Vr, thetar, omegar,
                 pot.value([robot.x, robot.y]))


# end of loop on simulation time


# close all figures
plt.close("all")

# generate plots
fig, ax = simu.plotXY(1)
# plot potential for verification of solution
pot.plot(noFigure=None, fig=fig, ax=ax)

simu.plotXYTheta(2)
simu.plotVOmega(3)

simu.plotPotential(4)


simu.plotPotential3D(5)


# show plots
#plt.show()


# Animation *********************************
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

thetaWPArea = np.arange(0.0, 2.0*math.pi+2*math.pi/30.0, 2.0*math.pi/30.0)
xWPArea = WPManager.epsilonWP*np.cos(thetaWPArea)
yWPArea = WPManager.epsilonWP*np.sin(thetaWPArea)


def initAnimation():
    robotDirection.set_data([], [])
    robotBody.set_data([], [])
    wayPoint.set_data([], [])
    WPArea.set_data([], [])
    robotBody.set_color('r')
    robotBody.set_markersize(20)
    time_text.set_text('')
    potential_text.set_text('')
    return robotBody, robotDirection, wayPoint, time_text, potential_text, WPArea


def animate(i):
    robotBody.set_data([simu.x[i]], [simu.y[i]])
    wayPoint.set_data([simu.xr[i]], [simu.yr[i]])
    WPArea.set_data([simu.xr[i]+xWPArea.transpose()],
                    [simu.yr[i]+yWPArea.transpose()])
    thisx = [simu.x[i], simu.x[i] + 0.5*math.cos(simu.theta[i])]
    thisy = [simu.y[i], simu.y[i] + 0.5*math.sin(simu.theta[i])]
    robotDirection.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*simu.dt))
    potential_text.set_text(potential_template %
                            (pot.value([simu.x[i], simu.y[i]])))
    return robotBody, robotDirection, wayPoint, time_text, potential_text, WPArea


ani = animation.FuncAnimation(fig, animate, np.arange(1, len(simu.t)),
                              interval=4, blit=True, init_func=initAnimation, repeat=False)
# interval=25

# ani.save('robot.mp4', fps=15)

plt.show()
