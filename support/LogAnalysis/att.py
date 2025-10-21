import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation as R

# function to generate desired angular velocity vectors for attitude control
# based on indi.c:getAlphaSpBody()
def get_w_des(theta, phi):
    # current attitude as quaternion using scipy.Rotation
    att = R.from_euler('ZYX', [0, theta, phi], degrees=False)

    # run code from indi.c:290
    Rerr = att.inv()
    targetZBody = Rerr.as_matrix()[:, 2]
    tiltErrorAngle = np.arccos(targetZBody[2])
    tiltAxisNorm = np.hypot(targetZBody[0], targetZBody[1])
    if tiltAxisNorm > 1e-6:
        tiltAxis = np.array([-targetZBody[1], targetZBody[0], 0.0]) / tiltAxisNorm
    else:
        if targetZBody[1] < 0.:
            tiltAxis = np.array([+1.0, 0.0, 0.0])
        else:
            tiltAxis = np.array([-1.0, 0.0, 0.0])

    tiltError = tiltAxis*tiltErrorAngle

    w_des = tiltError*2. # gain.. 1.0
#     w_des[0] *= 2.

    return w_des


w = get_w_des(np.pi, 170*np.pi/180)

fig = plt.figure(figsize=(7, 6))
ax = fig.add_subplot(111, projection='3d')

# Draw initial vector
vec = ax.quiver(0, 0, 0, w[0], w[1], w[2], color="r", linewidth=2)
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.view_init(elev=-16, azim=142, roll=180)

# --- Sliders ---
slider_ax_phi = plt.axes([0.2, 0.02, 0.65, 0.03])
slider_ax_theta = plt.axes([0.2, 0.06, 0.65, 0.03])

slider_phi = Slider(slider_ax_phi, "Phi", -360, 360, valinit=0)
slider_theta = Slider(slider_ax_theta, "Theta", -360, 360, valinit=0)

# --- Update function ---
def update(val):
    theta = slider_theta.val * np.pi/180
    phi = slider_phi.val * np.pi/180
    w = get_w_des(theta, phi)

    l = 0.5
    body = np.array([
        [    0,   2*l,     0.],
        [    0,   2*l,   -  l],
        [    0,    0.,   -2*l],
        [    0,  -2*l,   -  l],
        [    0,  -2*l,   0.],
        [    0,   2*l,   0.],
    ])

    body[:, 2] += l

    # rotate body with theta/phi using scipy
    att = R.from_euler('ZYX', [0, theta, phi], degrees=False)
    bodyI = att.apply(body)
    zB = att.apply(np.array([[0, 0, 1]]))

    wI = att.apply(w)

    vdes = np.cross(wI, -zB)
    print(np.linalg.norm(w))
    print(att.as_quat())

    # simulate trajectory
    dt = 0.02
    N = 201
    att_at = [20, 70, 200]
    zBi_traj = np.zeros((N, 3))
    zBi_traj[0] = zB.copy()
    wi = w.copy()
    atti = R.from_quat(att.as_quat())
    bodyI_traj = []
    for i in range(1, N):
        # tick attitude
        atti = atti * R.from_rotvec(wi*dt)

        # get new zBi from attitude
        zBi_traj[i] = atti.apply(np.array([[0, 0, 1]]))

        # get phi/theta
        atti_euler = atti.as_euler('ZYX', degrees=False)
        theta_i = atti_euler[1]
        phi_i = atti_euler[2]

        # get new w_des
        wi = get_w_des(theta_i, phi_i)

        if i in att_at:
            body_i = body.copy()
            bodyI_traj.append(atti.apply(body_i))

    # Clear and redraw
    ax.cla()
    ax.plot(bodyI[:, 0], bodyI[:, 1], bodyI[:, 2], color='b')
    for bi in bodyI_traj:
        ax.plot(bi[:, 0], bi[:, 1], bi[:, 2], color='c', alpha=0.3)
    ax.plot(-zBi_traj[:, 0]*l, -zBi_traj[:, 1]*l, -zBi_traj[:, 2]*l, color='g', linewidth=1)
    ax.quiver(0, 0, 0, wI[0], wI[1], wI[2], length=0.1*np.linalg.norm(wI), color="r", linewidth=2)
    ax.quiver(-zB[:, 0]*l, -zB[:, 1]*l, -zB[:, 2]*l,
              vdes[:, 0], vdes[:, 1], vdes[:, 2],
              length=0.2*np.linalg.norm(w), color='r', normalize=True)
    # ax.quiver(0, 0, 0, w[0], w[1], w[2], color="r", linewidth=2)
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect([1,1,1])  # Equal aspect ratio
    fig.canvas.draw_idle()

# Connect sliders to update
slider_theta.on_changed(update)
slider_phi.on_changed(update)


# plt.show()


# sample a grid on a unit sphere
num_linear = 21
num_samples = num_linear * (num_linear>>1)
phi = np.linspace(0, 2 * np.pi, num_linear)
theta = np.linspace(np.pi/(num_linear>>1), np.pi*((num_linear>>1)-1)/(num_linear>>1), num_linear >> 1)
phi, theta = np.meshgrid(phi, theta)
vdes = np.zeros((num_samples, 3))
zB = np.zeros((num_samples, 3))

i = 0
for phii, thetai in zip(phi.flatten(), theta.flatten()):
    w = get_w_des(thetai, phii)
    print(f"phi={phii*180/np.pi:.1f} deg, theta={thetai*180/np.pi:.1f} deg -> w={w}, norm={np.linalg.norm(w):.3f}")

    # rotate body with theta/phi using scipy
    att = R.from_euler('ZYX', [0, thetai, phii], degrees=False)
    zB[i] = att.apply(np.array([[0, 0, 1]]))

    wI = att.apply(w)

    vdes[i] = np.cross(wI, -zB[i])

    i += 1

# # plot unit sphere and for every zB, plot vdes
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
# ax.scatter(-zB[:, 0], -zB[:, 1], -zB[:, 2], color='b', s=1)
ax2.quiver(zB[:, 0], zB[:, 1], zB[:, 2],
          vdes[:, 0], vdes[:, 1], vdes[:, 2],
          length=0.2, color='r', normalize=True)
ax2.set_box_aspect([1,1,1])  # Equal aspect ratio

# label axes
ax2.set_xlim([-1, 1])
ax2.set_ylim([-1, 1])
ax2.set_zlim([-1, 1])
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

plt.show()


