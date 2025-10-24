# -*- coding: utf-8 -*-
"""
Way Point navigation - Phase 1 (circle scan with measurements + animation)

Implements the first hypothesis with improvements:
1) Go to a perimeter point of a circle centered at (0,0) (radius R)
2) Perform one full circle at constant speed while sampling the potential
3) Keep the position of the maximum measurement
4) Generate plots AND an animation GIF after the simulation

(c) S. Bertrand / Adapted for project steps
"""

import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential

# -------------------------
# Utility functions
# -------------------------
def wrap_to_pi(a):
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2.0*np.pi) - np.pi

# -------------------------
# Robot & Potential setup
# -------------------------
# robot initial state
x0 = -20.0
y0 = -20.0
theta0 = np.pi/4.0
robot = rob.Robot(x0, y0, theta0)

# potential field (pollutant)
pot = Potential.Potential(difficulty=1, random=True)

# control loop gains and timers
kpPos = 0.8
positionCtrlPeriod = 0.2
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

kpOrient = 2.5
orientationCtrlPeriod = 0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)

# dummy WP manager (not used for control here, but kept for plotting compatibility)
WPlist = [[x0, y0]]
epsilonWP = 1
WPManager = rob.WPManager(WPlist, epsilonWP)

# simulation horizon
t0 = 0.0
tf = 140.0   # slightly longer to ensure we finish the lap even from far start
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)

# -------------------------
# Phase 1 parameters
# -------------------------
CENTER_X, CENTER_Y = 0.0, 0.0       # circle center
CIRCLE_R = 5.0                      # circle radius (meters)
V_MAX = 3.0                         # max linear speed (m/s)
OMEGA_MAX = 2.0                     # max angular speed (rad/s)
V_CIRC = 0.6                        # linear speed during circle (m/s)
OMEGA_CIRC = V_CIRC / CIRCLE_R      # angular speed to track circle
DIST_TOL_TARGET = 2.5              # tolerance to reach the perimeter point
ONE_LAP = 2.0*np.pi                 # circle completion threshold
direction = -1.0                    # +1 for CCW rotation

# choose a fixed starting angle for perimeter point (here pi -> (-R,0))
start_angle = np.pi
perimeter_target = (CENTER_X + CIRCLE_R*np.cos(start_angle),
                    CENTER_Y + CIRCLE_R*np.sin(start_angle))

# -------------------------
# State machine
# -------------------------
state = "goto_perimeter"            # "goto_perimeter" -> "circle" -> "stop"
lap_angle_acc = 0.0                 # accumulated absolute angular travel around center
prev_angle = None                   # previous bearing angle from center

# storage for sampling during circle
sample_positions = []               # list of (x,y)
sample_potentials = []              # list of potential values
max_potential = -np.inf
max_position = (None, None)

# control inputs
Vr = 0.0
thetar = robot.theta
omegar = 0.0

# -------------------------
# Simulation loop
# -------------------------
for t in simu.t:

    if timerPositionCtrl.isEllapsed(t):
        if state == "goto_perimeter":
            # target is the chosen perimeter point on the circle
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
                # set constant speeds for circle following
                Vr = V_CIRC
                omegar = direction * OMEGA_CIRC

        elif state == "circle":
            # constant circular motion
            Vr = V_CIRC
            omegar = direction * OMEGA_CIRC

            # accumulate lap angle
            angle_now = math.atan2(robot.y - CENTER_Y, robot.x - CENTER_X)
            if prev_angle is not None:
                d_ang = wrap_to_pi(angle_now - prev_angle)
                lap_angle_acc += abs(d_ang)
            prev_angle = angle_now

            # sample potential
            pval = pot.value([robot.x, robot.y])
            sample_positions.append((robot.x, robot.y))
            sample_potentials.append(pval)
            if pval > max_potential:
                max_potential = pval
                max_position = (robot.x, robot.y)

            # stop after one full lap
            if lap_angle_acc >= ONE_LAP:
                state = "stop"
                Vr = 0.0
                omegar = 0.0

        elif state == "stop":
            Vr = 0.0
            omegar = 0.0
            thetar = robot.theta

    # orientation loop (not adding a second computation layer)
    if timerOrientationCtrl.isEllapsed(t):
        pass

    # apply control & integrate
    robot.setV(Vr)
    robot.setOmega(omegar)
    robot.integrateMotion(dt)

    # log
    simu.addData(robot, WPManager, Vr, thetar, omegar, pot.value([robot.x, robot.y]))

# -------------------------
# Static Plots
# -------------------------
plt.close("all")

# 1) Trajectory + potential field
fig, ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)

# overlay circle, center, perimeter target
circ = plt.Circle((CENTER_X, CENTER_Y), CIRCLE_R, fill=False, linestyle='--', linewidth=1.5, color='k', alpha=0.9)
ax.add_patch(circ)
ax.scatter([CENTER_X], [CENTER_Y], marker='x', s=60, c='k', label='Center')
ax.plot([perimeter_target[0]], [perimeter_target[1]], 'bo', markersize=5, label='Circle start')

# highlight max sample
if max_position[0] is not None:
    ax.plot(max_position[0], max_position[1], marker='*', markersize=18, markeredgecolor='k', markerfacecolor='yellow', linestyle='None', label='Max potential')
ax.legend(loc='upper right')
ax.set_title("Phase 1: circle scan (R=%.1fm) — max at (%.2f, %.2f), value=%.2f" % (CIRCLE_R, max_position[0], max_position[1], max_potential))

# 2) Time plots
simu.plotXYTheta(2)
simu.plotPotential(3)
simu.plotPotential3D(4)

# -------------------------
# Animation (post-simulation)
# -------------------------
# Build arrays for animation
xs = simu.x
ys = simu.y
ths = simu.theta
pots = simu.potential
ts = simu.t

# Determine axis limits
xmin, xmax = -25, 25
ymin, ymax = -25, 25

fig_anim = plt.figure(5, figsize=(8.5, 6))
# Main axis for robot motion
ax_main = fig_anim.add_subplot(1, 1, 1, aspect='equal', autoscale_on=False, xlim=(xmin, xmax), ylim=(ymin, ymax))
ax_main.grid(True, linestyle=':')
ax_main.set_xlabel('x (m)')
ax_main.set_ylabel('y (m)')
ax_main.set_title('Circle scan animation — R=%.1fm' % CIRCLE_R)

# plot potential field as static background contours
pot.plot(noFigure=None, fig=fig_anim, ax=ax_main)

# draw circle & center
circ_anim = plt.Circle((CENTER_X, CENTER_Y), CIRCLE_R, fill=False, linestyle='--', linewidth=1.5, color='k', alpha=0.9)
ax_main.add_patch(circ_anim)
ax_main.scatter([CENTER_X], [CENTER_Y], marker='x', s=60, c='k')

# draw max point (static, known post-sim)
if max_position[0] is not None:
    ax_main.plot(max_position[0], max_position[1], marker='*', markersize=14, markeredgecolor='k', markerfacecolor='yellow', linestyle='None')

# elements to update
robotBody, = ax_main.plot([], [], 'o', lw=0, color='r', markersize=8)
robotDirection, = ax_main.plot([], [], '-', lw=1, color='k')
trace, = ax_main.plot([], [], '-', lw=2, color='r', alpha=0.8)

# --- Side colorbar (dynamic indicator of current potential) ---
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

pot_vals = np.array([p for p in pots if not np.isnan(p)])
if pot_vals.size == 0:
    pot_min, pot_max = -10.0, 10.0
else:
    pot_min, pot_max = float(np.min(pot_vals)), float(np.max(pot_vals))

# Create an inset axis for the colorbar
cax = inset_axes(ax_main, width="2.5%", height="60%", loc='center right', borderpad=2)
norm = Normalize(vmin=pot_min, vmax=pot_max)
sm = ScalarMappable(norm=norm, cmap='plasma')
cb = plt.colorbar(sm, cax=cax)
cb.set_label('Potential')

# Marker on the colorbar indicating current potential
cb_marker, = cax.plot([0,1], [0,0], '-', lw=3, color='k')
cax.set_ylim(pot_min, pot_max)

time_template = 't = %.1fs'
time_text = ax_main.text(0.02, 0.95, '', transform=ax_main.transAxes)

# Circle start marker
ax_main.plot([perimeter_target[0]], [perimeter_target[1]], 'bo', markersize=5)

# Frames (subsample for speed)
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
    time_text.set_text(time_template % (i*dt))
    p = pots[i]
    if not np.isnan(p):
        cb_marker.set_data([0,1], [p, p])
    return robotBody, robotDirection, trace, time_text, cb_marker

ani = animation.FuncAnimation(fig_anim, animate, frames_to_animate,
                              interval=25, blit=True, init_func=initAnimation, repeat=True)

# Save GIF
ani.save('robot_circle_scan.gif', writer='pillow', fps=20, dpi=100)

plt.show()
