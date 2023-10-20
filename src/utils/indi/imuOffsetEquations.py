from sage.all import *


fvars = var('f_x f_y f_z')
wvars = var('w_x w_y w_z')
avars = var('a_x a_y a_z')
rvars = var('r_x r_y r_z')

[assume(x, 'real') for x in fvars]
[assume(x, 'real') for x in wvars]
[assume(x, 'real') for x in avars]
[assume(x, 'real') for x in rvars]

f = vector((f_x, f_y, f_z))
w = vector((w_x, w_y, w_z))
a = vector((a_x, a_y, a_z))
r = vector((r_x, r_y, r_z))

RHS = a.cross_product(r)  +  w.cross_product(w.cross_product(r))

A = Matrix(SR, 3, 3)
for i in range(3):
    for j, v in enumerate(rvars):
        A[i, j] = RHS[i].coefficients(v)[1][0]

print("Ax = b")
print(A)
print(r)
print("=")
print(f)

print()

print(latex(A))
print(latex(r))
print("=")
print(latex(f))

