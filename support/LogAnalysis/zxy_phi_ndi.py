#!/usr/bin/env python3
"""
Symbolic derivation of the Jacobian, with respect to the quaternion components,
of

    f(q) = q^-1 ( A ( q v q^-1 ) ) q  +  s ( q a q^-1 )

for a unit quaternion q = (q0, q1, q2, q3) (q0 the scalar part, Hamilton
convention), a positive-definite 3x3 matrix A, 3-vectors a and v, and scalar s.

Interpretation used here (juxtaposition of a quaternion and a 3-vector denotes
the usual sandwich rotation):

    Rot(q, x) := Vec( q (0,x) q^-1 )

  - term1 = q^-1 A q v
          = Rot( q^-1, A @ Rot(q, v) )
          i.e. rotate v into q's frame, apply the gain matrix A, then rotate
          the result back out by q^-1. (Equivalent to embedding each column of
          A as a pure quaternion and rotating those columns instead of
          rotating v then multiplying by A -- both give the same result since
          the sandwich rotation is linear in its vector argument.)
  - term2 = s q a q^-1 = s * Rot(q, a)

Since q is a unit quaternion, q^-1 == conjugate(q), which is used throughout
(no explicit unit-norm constraint is substituted into the derivatives -- this
matches how such Jacobians are normally used, e.g. in attitude filters, where
q's four components are differentiated as free variables).

Both terms are pure quaternions (zero scalar part) for any vector argument, so
f(q) is effectively R^3-valued; the returned Jacobian is 3x4 (rows = output
components x,y,z, columns = dq0..dq3).
"""

import re

import sympy as sp

def build_rotmat(axis, angle):
    sa = sp.sin(angle)
    ca = sp.cos(angle)

    R = sp.eye(3)

    if axis == 0:
        R[1,1] = R[2,2] = ca
        R[1,2] = -sa
        R[2,1] = sa
    elif axis == 1:
        R[2,2] = R[0,0] = ca
        R[2,0] = -sa
        R[0,2] = sa
    elif axis == 2:
        R[0,0] = R[1,1] = ca
        R[0,1] = -sa
        R[1,0] = sa

    return R

def get_jacobian(expr, variables, cse=False):
    f = sp.expand(expr)
    J = sp.expand(f.jacobian(variables))
    J = sp.simplify(J)

    # f and J are quartic in q (two nested rotations), so the fully expanded
    # form has far too many terms to read directly. CSE brings it down to a
    # manageable set of shared subexpressions.
    def cse_or_not(exprs):
        if cse:
            return sp.cse(list(exprs), symbols=sp.numbered_symbols('t'))
        return [], list(exprs)

    print("--- f(q)" + (", CSE'd" if cse else "") + " ---\n")
    repl, reduced = cse_or_not(f)
    for sym, expr in repl:
        print(f"  {sym} = {expr}")
    for i, expr in enumerate(reduced):
        print(f"  f[{i}] = {expr}")

    print("\n--- Jacobian df/dq (rows x,y,z ; columns q0,q1,q2,q3)" + (", CSE'd" if cse else "") + " ---\n")
    repl, reduced = cse_or_not(J)
    for sym, expr in repl:
        print(f"  {sym} = {expr}")
    for idx, expr in enumerate(reduced):
        row, col = divmod(idx, J.shape[1])
        print(f"  J[{row}][{col}] = {expr}")

    return J

def emit_attitude_thrust_jacobian(J, substitutions, cse=True):
    """Print C for a supplied attitude/thrust Jacobian and symbol mapping."""
    J = sp.Matrix(J)
    # if Jtotal.shape != (3, 4):
    #     raise ValueError(f'Expected a 3x4 Jtotal, got {Jtotal.shape}')

    if cse:
        replacements, reduced = sp.cse(
            list(J), symbols=sp.numbered_symbols('t'), order='canonical'
        )
    else:
        replacements, reduced = [], list(J)

    def c_expression(expr):
        code = sp.ccode(expr)
        for symbol, c_name in substitutions.items():
            code = re.sub(rf'\b{re.escape(str(symbol))}\b', c_name, code)
        return re.sub(
            r'(?<![\w.])(\d+\.\d+(?:[eE][+-]?\d+)?)(?![\w.])',
            r'\1f',
            code,
        )

    print('void getAttitudeThrustJacobian(')
    print('    float** J, const float** A_B, const fp_vector_t* ax_B,')
    print('    const fp_quaternion_t* q_0, const fp_vector_t* f_B_0,')
    print('    const fp_vector_t* v_I, const float V)')
    print('{')
    for symbol, expr in replacements:
        print(f'    const float {symbol} = {c_expression(expr)};')
    for idx, expr in enumerate(reduced):
        row, col = divmod(idx, J.cols)
        print(f'    J[{row}][{col}] = {c_expression(expr)};')
    print('}')


if __name__ == '__main__':
    fix, fiy, fiz = sp.symbols('fix fiy fiz', real=True)
    fi_symbols = fix, fiy, fiz
    f_I = sp.Matrix([fix, fiy, fiz])

    # positive-definite matrices are symmetric
    a11, a12, a13, a22, a23, a33 = sp.symbols('a11 a12 a13 a22 a23 a33', real=True)
    a_symbols = a11, a12, a13, a22, a23, a33
    PHI = sp.Matrix([ # only conventional lift and drag in the xz plane
        [a11, 0, 0],
        [0, 0, 0],
        [0, 0, a33],
    ])

    vIx, vIy, vIz = sp.symbols('vIx vIy vIz', real=True)
    v_symbols = vIx, vIy, vIz
    axx, axy, axz = sp.symbols('axx axy axz', real=True)
    ax_symbols = axx, axy, axz

    V = sp.symbols('V', real=True)
    v_I = sp.Matrix([vIx, vIy, vIz])
    ax_B = sp.Matrix([axx, 0, axz])

    phi, theta, psi = sp.symbols('phi theta psi', real=True)
    R_psi = build_rotmat(2, psi)
    R_theta = build_rotmat(1, theta)
    R_phi = build_rotmat(0, phi)
    R_total = R_psi*R_phi*R_theta
    R_no_psi = R_phi*R_theta

    T = sp.symbols('T', real=True)
    # expr_R  =  R @ A @ R.T  @ v_I   +   R @ ax_B * T


    # lets get the eulers ZXY convention (from Ezra Tal)
    # Psi is given
    # phi can be calcuated by assuming that we cannot generate forces in the body-y direction
    #     [0 1 0] @ R_total.T @ f_I  ==  0
    #     This means that all lateral force in the R_psi frame has to be generated by rolling to align the body xz plane in which we _can_ generate forces
    #     This means the resultant of desired fy+fz in R_psi frame (f_psi = R_psi.T @ f_I) needs to point into the same direction as the resulting z axis after R_phi

    f_psi = R_psi.T @ f_I
    expr_R_psi = sp.simplify( -V * R_phi @ R_theta @ PHI @ R_total.T  @ v_I   +   R_phi @ R_theta @ ax_B * T )
    phi_star = sp.solve( sp.simplify(expr_R_psi[1] / expr_R_psi[2]) - f_psi[1] / f_psi[2], phi)[0]

    # We now know R_psi@R_phi. Therefore  R_theta.T (R_psi@R_phi).T @ f_I  are the body forces!
    # The body forces are expressed simply as  -V * PHI @ R_theta.T @ (R_psi@R_phi).T @ v_I  +  ax_B * T
    # Simplify notation by defining
    #    f_phi = (R_psi@R_phi).T @ f_I
    #    v_phi = (R_psi@R_phi).T @ v_I
    fpx, fpy, fpz = sp.symbols('fpx fpy fpz', real=True)
    f_phi = sp.Matrix([fpz, fpy, fpz])
    f_body = R_theta.T @ f_phi
    vpx, vpy, vpz = sp.symbols('vpx vpy vpz', real=True)
    v_phi = sp.Matrix([vpx, vpy, vpz])

    # body forces
    expr_body  = sp.simplify( -V * PHI @ R_theta.T @ v_phi  +  ax_B * T )

    # to solve:  expr_body == f_body
    # we're only interested in x and z components.
    # solve manually by eliminating T, then atan2 should show up naturally because division of sin/cos
    x_for_T = sp.solve(expr_body[0], T)[0]
    eq_theta = expr_body[2].subs(T, x_for_T)
    sin_cos_terms = sp.collect(sp.expand_trig(eq_theta).expand(), [sp.cos(theta), sp.sin(theta)], evaluate=False)


