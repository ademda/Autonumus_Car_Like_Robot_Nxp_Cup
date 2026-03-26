import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Robot parameters
# -----------------------------
L = 0.5         # wheelbase
max_steer = np.radians(30)

# -----------------------------
# Goal position
# -----------------------------
x_goal = 5.0
y_goal = 5.0

# -----------------------------
# Controller gains
# -----------------------------
k_r = 1.0
k_y = 2.0

# -----------------------------
# Simulation parameters
# -----------------------------
T = 20
dt = 0.05
t = np.arange(0, T, dt)

# -----------------------------
# Robot initial state
# -----------------------------
x = 0.0
y = -5.0
theta = 0.0

x_hist = []
y_hist = []
theta_hist = []
delta_hist = []

# -----------------------------
# Simulation loop
# -----------------------------
for i in range(len(t)):
    # Compute errors
    dx = x_goal - x
    dy = y_goal - y
    r = np.sqrt(dx**2 + dy**2)             # distance to goal
    theta_des = np.arctan2(dy, dx)         # angle to goal
    theta_e = theta_des - theta
    theta_e = np.arctan2(np.sin(theta_e), np.cos(theta_e))  # wrap [-pi,pi]

    # Lateral error in robot frame
    y_r = -np.sin(theta)*dx + np.cos(theta)*dy

    # Time-invariant Stanley-like controller
    v = k_r * r
    delta = theta_e + np.arctan(k_y * y_r)

    # Saturate steering
    delta = np.clip(delta, -max_steer, max_steer)

    # Ackermann kinematics
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += v / L * np.tan(delta) * dt

    # Store history
    x_hist.append(x)
    y_hist.append(y)
    theta_hist.append(theta)
    delta_hist.append(delta)

# -----------------------------
# Plot results
# -----------------------------
plt.figure()
plt.plot(x_goal, y_goal, 'ro', label='Goal')
plt.plot(x_hist, y_hist, 'b', label='Robot path')
plt.axis('equal')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()
plt.title('Time-invariant Stanley-like Controller for Point Stabilization')
plt.show()

plt.figure()
plt.plot(t, delta_hist)
plt.xlabel('Time [s]')
plt.ylabel('Steering angle [rad]')
plt.title('Steering angle over time')
plt.show()
