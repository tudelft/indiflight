from sage.all import *

# assume the following rotation sequence,
#   and a north east down inertial frame
#   and a fwd-right-down body frame
# 
# first: rotate around an axis-angle rotation with the z component = 0 (ie the tilt)
# second: rotate around inertial z-axis (yaw)

# inertial accel a can then be expressed as a function of specific body forces 
# f and gravity G = 9.81
#
#   a^I  =  ( Rz(Psi) * axang(alpha, axis) )  @  f^B   +   (0 0 G)
#       where axis = (tx ty 0)

# reformulate:
#   v^I  = Rz^{-1}(Psi) @ (a^I - g^I) = axang(alpha, axis)  @  f^B

# now, also assume f^B = (0 0 -fz)

vars = var('vx vy vz ax ay az Psi alpha tx ty fz G')
[assume(a, "real") for a in vars]
assume(alpha >= 0)


eqs = [
    1 == (tx**2 + ty**2),
    vx == ty * sin(alpha) * (-fz),
    vy == -tx * sin(alpha) * (-fz),
    vz == cos(alpha) * (-fz),
]

