from sage.all import *

Ivars = var('I_xx I_yy I_zz I_xy I_xz I_yz')
wvars = var('w_x w_y w_z')
avars = var('a_x a_y a_z')

[assume(x, 'real') for x in Ivars]
[assume(x, 'real') for x in wvars]
[assume(x, 'real') for x in avars]
assume(I_xx > 0)
assume(I_yy > 0)
assume(I_zz > 0)

I = Matrix([[I_xx, I_xy, I_xz], [I_xy, I_yy, I_yz], [I_xz, I_yz, I_zz]])
Ivec = vector((I_xx, I_yy, I_zz, I_xy, I_xz, I_yz))
w = vector((w_x, w_y, w_z))
a = vector((a_x, a_y, a_z))

RHS = I * a  +  w.cross_product(I * w)

A = Matrix(SR, 3, 6) # SR: symbolic
for i in range(3):
    for j, v in enumerate(Ivec):
        A[i, j] = RHS[i].coefficients(v)[1][0]

print("Ax = 0")
print(A)
print(Ivec)
print("= 0")

print()

print(latex(A))
print(latex(Ivec))
print("= 0")
