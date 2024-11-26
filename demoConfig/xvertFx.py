import numpy as np

b = 0.5
mac = 0.15
m_frame = 65e-3
m_servo = 20e-3
m_motor = 42e-3

y_servo = 0.05
z_servo = 0.05

x_motor = [0., 0.]
y_motor = [-0.15, 0.15]
z_motor = [0., 0.]

mdir = [-1, 1]


lam = 0.5 # flap flow deflection ratio

m_total = 300e-3
D_prop = 5*25.4e-3
A_prop = np.pi * D_prop**2 / 4
rho = 1.225
mdot_hover = lam

G = 9.81
T_hover = m_total * G

D2R = np.pi / 180.
delta_servo_max = 60 * D2R
delta_flap_max = 40 * D2R
flap_CP_z = 0.12
flap_CP_y = y_motor[1]

M_pitch_flap_max = np.sin(delta_flap_max) * lam * 0.5*T_hover * flap_CP_z
M_yaw_flap_max = np.sin(delta_flap_max) * lam * 0.5*T_hover * flap_CP_y

#M_roll_motor_max = 
#F_motor_max = 

T_max_race = 100 * 500e-3 / 4 # race drone can do 10g with 500gram mass and similar props
rpm_max_race = 2400 * 4 #2400kv with 4S
rpm_max = 1750 * 4 * 0.95 #1750kv with 4S and higher voltage drop
T_max = T_max_race * (rpm_max / rpm_max_race)**2


F1 = np.zeros((6, 4))
F1[2, :2] = -T_max
F1[3, :2] = -np.asarray(y_motor) * T_max
F1[4, 2:] = (M_pitch_flap_max, -M_pitch_flap_max)
F1[5, :2] = T_max * -np.asarray(mdir) * 0.002
F1[5, 2:] = (M_yaw_flap_max, M_yaw_flap_max)

m_bat = 65e-3
Ixx = 1/12 * (b**2 + mac**2) * m_frame  +  2 * (m_motor * y_motor[1]**2 + m_servo * y_servo**2) + m_bat * 0.05**2
Iyy = 1/12 * mac**2 * m_frame  +  2 * (m_motor * 0.02**2 + m_servo * 0.05**2) + m_bat * 0.05**2
Izz = 1/12 * b**2 * m_frame  +  2 * (m_motor * y_motor[1]**2 + m_servo * y_servo**2)

fudge = 1.25
Iyy_fudge = 2.

M = np.diag((m_total, m_total, m_total, Ixx*fudge, Iyy*fudge*Iyy_fudge, Izz*fudge))

G1 = np.linalg.inv(M) @ F1
print(G1)
print((G1 * np.array([[100, 100, 100, 10, 10, 10]]).T).round(0))
