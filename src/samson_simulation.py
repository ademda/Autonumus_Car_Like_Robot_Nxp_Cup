import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Trajectory generation (circle)
# -----------------------------
T = 10       # total time [s]
dt = 0.01     # time step [s]
t = np.arange(0, T, dt)

R = 20        # circle radius
omega = 0.2  # angular speed [rad/s]

#xd = R * np.exp(omega * t)
#yd = R * t
xd = R * np.sin(omega * t)
yd = R * np.cos(omega*t)
theta_d = np.arctan2(np.gradient(yd, dt), np.gradient(xd, dt))
vd = np.sqrt(np.gradient(xd, dt)**2 + np.gradient(yd, dt)**2)

# -----------------------------
# Samson controller gains
# -----------------------------
k1 = 8.0
k2 = 0.8

# -----------------------------
# Car parameters
# -----------------------------
L = 0.5  # wheelbase [m]

# -----------------------------
# Robot initialization
# -----------------------------
x = -5.0
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
    # Errors in robot frame
    ex = np.cos(theta)*(xd[i]-x) + np.sin(theta)*(yd[i]-y)
    ey = -np.sin(theta)*(xd[i]-x) + np.cos(theta)*(yd[i]-y)
    etheta = theta_d[i] - theta

    # Samson control law (rear-wheel drive + steering)
    v = vd[i] * np.cos(etheta) + k1 * ex
    omega = k2 * vd[i] * ey + k1 * np.sin(etheta)

    # Convert to steering angle
    delta = np.arctan2(L * omega, v + 1e-5)  # avoid div by zero

    # Car kinematics
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += v / L * np.tan(delta) * dt

    x_hist.append(x)
    y_hist.append(y)
    theta_hist.append(theta)
    delta_hist.append(delta)

# -----------------------------
# Plot results
# -----------------------------
plt.figure()
plt.plot(xd, yd, 'r--', label='Desired trajectory')
plt.plot(x_hist, y_hist, 'b', label='Robot path')
plt.axis('equal')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()
plt.title('Samson Controller for Car-like Robot')
plt.show()

plt.figure()
plt.plot(t, delta_hist)
plt.xlabel('Time [s]')
plt.ylabel('Steering angle [rad]')
plt.title('Steering angle over time')
plt.show()
