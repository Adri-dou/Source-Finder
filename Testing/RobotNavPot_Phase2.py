# -*- coding: utf-8 -*-
"""
Robot navigation — Phase 1 + Phase 2

Phase 1 (exploration):
  - Go to perimeter of a circle centered at (0,0) with radius R
  - Perform one full clockwise circle at constant speed
  - Sample the potential and store max value/position

Phase 2 (real-time isopotential following):
  - After circle, advance at constant speed
  - Adjust angular velocity to stay close to isopotential V = Vmax - 20
  - Smooth control (low gain), stop after 60 seconds of Phase 2

Also:
  - Preserve static plots
  - Build an animation (with trace, direction, circle, center, and star at max)
  - Save GIF at 20 fps: robot_circle_scan.gif

(c) S. Bertrand / Adapted for project steps
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import Robot as rob
import Timer as tmr
import Potential

def wrap_to_pi(a):
    return (a + np.pi) % (2.0*np.pi) - np.pi

# Robot & potential
x0, y0, theta0 = -20.0, -20.0, np.pi/4.0
robot = rob.Robot(x0, y0, theta0)
pot = Potential.Potential(difficulty=1, random=False)

# Control timers
kpPos = 0.8
kpOrient = 2.5
positionCtrlPeriod = 0.2
orientationCtrlPeriod = 0.05
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)

# WP manager for logging
WPlist = [[x0, y0]]
epsilonWP = 0.2
WPManager = rob.WPManager(WPlist, epsilonWP)

# Simulation timing
t0, tf, dt = 0.0, 220.0, 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)

# Phase parameters
CENTER_X, CENTER_Y = 0.0, 0.0
CIRCLE_R = 5.0
V_MAX, OMEGA_MAX = 1.0, 2.0
V_CIRC = 0.6
OMEGA_CIRC = V_CIRC / CIRCLE_R
DIST_TOL_TARGET = 0.25
ONE_LAP = 2.0*np.pi
direction = -1.0  # clockwise

start_angle = np.pi  # (-R, 0)
perimeter_target = (CENTER_X + CIRCLE_R*np.cos(start_angle),
                    CENTER_Y + CIRCLE_R*np.sin(start_angle))

# Phase 2 control
V_ISO = 0.5
K_iso = 0.01
target_potential = None
phase2_time = 0.0
PHASE2_MAX_TIME = 60.0

# State machine
state = "goto_perimeter"
lap_angle_acc = 0.0
prev_angle = None

sample_positions = []
sample_potentials = []
max_potential = -np.inf
max_position = (None, None)

Vr, thetar, omegar = 0.0, robot.theta, 0.0

# Simulation loop
for t in simu.t:
    if timerPositionCtrl.isEllapsed(t):

        if state == "goto_perimeter":
            dx = perimeter_target[0] - robot.x
            dy = perimeter_target[1] - robot.y
            dist = math.hypot(dx, dy)
            thetar = math.atan2(dy, dx)
            ang_err = wrap_to_pi(thetar - robot.theta)
            Vr = min(kpPos * dist, V_MAX)
            omegar = np.clip(kpOrient * ang_err, -OMEGA_MAX, OMEGA_MAX)
            if dist < DIST_TOL_TARGET:
                state = "circle"
                prev_angle = math.atan2(robot.y - CENTER_Y, robot.x - CENTER_X)
                lap_angle_acc = 0.0
                Vr = V_CIRC
                omegar = direction * OMEGA_CIRC

        elif state == "circle":
            Vr = V_CIRC
            omegar = direction * OMEGA_CIRC
            angle_now = math.atan2(robot.y - CENTER_Y, robot.x - CENTER_X)
            if prev_angle is not None:
                d_ang = wrap_to_pi(angle_now - prev_angle)
                lap_angle_acc += abs(d_ang)
            prev_angle = angle_now
            pval = pot.value([robot.x, robot.y])
            sample_positions.append((robot.x, robot.y))
            sample_potentials.append(pval)
            if pval > max_potential:
                max_potential = pval
                max_position = (robot.x, robot.y)
            if lap_angle_acc >= ONE_LAP:
                state = "follow_isopotential"
                target_potential = max_potential - 20.0
                phase2_time = 0.0

        elif state == "follow_isopotential":
            Vr = V_ISO
            pval = pot.value([robot.x, robot.y])
            e = pval - target_potential
            omegar = -K_iso * e
            phase2_time += positionCtrlPeriod
            if phase2_time >= PHASE2_MAX_TIME:
                state = "stop"
                Vr = 0.0
                omegar = 0.0

        elif state == "stop":
            Vr = 0.0
            omegar = 0.0
            thetar = robot.theta

    if timerOrientationCtrl.isEllapsed(t):
        pass

    robot.setV(Vr)
    robot.setOmega(omegar)
    robot.integrateMotion(dt)
    simu.addData(robot, WPManager, Vr, thetar, omegar, pot.value([robot.x, robot.y]))

# -------- Static plots --------
plt.close("all")
fig, ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)
circ = plt.Circle((CENTER_X, CENTER_Y), CIRCLE_R, fill=False, linestyle='--', linewidth=1.5, color='k', alpha=0.9)
ax.add_patch(circ)
ax.scatter([CENTER_X], [CENTER_Y], marker='x', s=60, c='k', label='Center')
ax.plot([perimeter_target[0]], [perimeter_target[1]], 'bo', markersize=5, label='Circle start')
if max_position[0] is not None:
    ax.plot([max_position[0], max_position[1]], marker='*', markersize=18, markeredgecolor='k', markerfacecolor='yellow', linestyle='None', label='Max potential')
ax.legend(loc='upper right')
ax.set_title("Phase 1+2: circle scan (R=%.1fm) then isopotential follow — Vmax=%.2f, target=%.2f" % (CIRCLE_R, max_potential, (max_potential-20.0)))
simu.plotXYTheta(2)
simu.plotPotential(3)
simu.plotPotential3D(4)

# -------- Animation --------
xs, ys, ths, pots, ts = simu.x, simu.y, simu.theta, simu.potential, simu.t
xmin, xmax = -20, 20
ymin, ymax = -20, 20
fig_anim = plt.figure(5, figsize=(8.5, 6))
ax_main = fig_anim.add_subplot(1, 1, 1, aspect='equal', autoscale_on=False, xlim=(xmin, xmax), ylim=(ymin, ymax))
ax_main.grid(True, linestyle=':')
ax_main.set_xlabel('x (m)')
ax_main.set_ylabel('y (m)')
ax_main.set_title('Circle scan + isopotential follow (GIF @20fps)')
pot.plot(noFigure=None, fig=fig_anim, ax=ax_main)
circ_anim = plt.Circle((CENTER_X, CENTER_Y), CIRCLE_R, fill=False, linestyle='--', linewidth=1.5, color='k', alpha=0.9)
ax_main.add_patch(circ_anim)
ax_main.scatter([CENTER_X], [CENTER_Y], marker='x', s=60, c='k')
if max_position[0] is not None:
    ax_main.plot(max_position[0], max_position[1], marker='*', markersize=14, markeredgecolor='k', markerfacecolor='yellow', linestyle='None')
ax_main.plot([perimeter_target[0]], [perimeter_target[1]], 'bo', markersize=5)
robotBody, = ax_main.plot([], [], 'o', lw=0, color='r', markersize=8)
robotDirection, = ax_main.plot([], [], '-', lw=1, color='k')
trace, = ax_main.plot([], [], '-', lw=2, color='r', alpha=0.85)

from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
pot_vals = np.array([p for p in pots if not np.isnan(p)])
if pot_vals.size == 0:
    pot_min, pot_max = -10.0, 10.0
else:
    pot_min, pot_max = float(np.min(pot_vals)), float(np.max(pot_vals))
cax = inset_axes(ax_main, width="2.5%", height="60%", loc='center right', borderpad=2)
norm = Normalize(vmin=pot_min, vmax=pot_max)
sm = ScalarMappable(norm=norm, cmap='plasma')
cb = plt.colorbar(sm, cax=cax)
cb.set_label('Potential')
cb_marker, = cax.plot([0,1], [0,0], '-', lw=3, color='k')
cax.set_ylim(pot_min, pot_max)
time_template = 't = %.1fs'
time_text = ax_main.text(0.02, 0.95, '', transform=ax_main.transAxes)
frame_skip = 5
frames_to_animate = np.arange(1, len(ts), frame_skip)
def initAnimation():
    robotBody.set_data([], [])
    robotDirection.set_data([], [])
    trace.set_data([], [])
    time_text.set_text('')
    cb_marker.set_data([0,1], [pot_min, pot_min])
    return robotBody, robotDirection, trace, time_text, cb_marker
def animate(i):
    trace.set_data(xs[:i], ys[:i])
    robotBody.set_data([xs[i]], [ys[i]])
    thisx = [xs[i], xs[i] + 0.6*math.cos(ths[i])]
    thisy = [ys[i], ys[i] + 0.6*math.sin(ths[i])]
    robotDirection.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*0.01))
    p = pots[i]
    if not np.isnan(p):
        cb_marker.set_data([0,1], [p, p])
    return robotBody, robotDirection, trace, time_text, cb_marker
ani = animation.FuncAnimation(fig_anim, animate, frames_to_animate, interval=25, blit=True, init_func=initAnimation, repeat=True)
ani.save('robot_circle_scan.gif', writer='pillow', fps=20, dpi=100)
plt.show()
