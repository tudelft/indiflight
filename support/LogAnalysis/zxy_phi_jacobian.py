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
import sys

import sympy as sp

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

    # print("\n--- Same Jacobian, as C code ---\n")
    # repl, reduced = cse_or_not(J)
    # for sym, expr in repl:
    #     print(f"  const float {sym} = {sp.ccode(expr)};")
    # for idx, expr in enumerate(reduced):
    #     row, col = divmod(idx, J.shape[1])
    #     print(f"  J[{row}][{col}] = {sp.ccode(expr)};")


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

def quat_mul(p, q):
    """Hamilton product of two quaternions given as 4-tuples (w, x, y, z)."""
    p0, p1, p2, p3 = p
    q0, q1, q2, q3 = q
    return (
        p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
        p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
        p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
        p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0,
    )


def quat_conj(q):
    q0, q1, q2, q3 = q
    return (q0, -q1, -q2, -q3)


def pure(v):
    """Embed a 3-vector as a pure quaternion (zero scalar part)."""
    return (sp.Integer(0), v[0], v[1], v[2])


def rotate(q, v):
    """Rot(q, v) = Vec( q (0,v) q^-1 ), returned as a length-3 tuple."""
    rotated = quat_mul(quat_mul(q, pure(v)), quat_conj(q))
    return rotated[1:]


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

    qw, qx, qy, qz = sp.symbols('qw qx qy qz', real=True)
    q_symbols = qw, qx, qy, qz
    # q = (qw, qx, qy, qz)
    # qi = quat_conj(q)  # valid since q is a unit quaternion

    q = sp.Quaternion(qw, qx, qy, qz, norm=1)
    qi = q.inverse()

    # positive-definite matrices are symmetric
    a11, a12, a13, a22, a23, a33 = sp.symbols('a11 a12 a13 a22 a23 a33', real=True)
    a_symbols = a11, a12, a13, a22, a23, a33
    # A = sp.Matrix([ # full
    #     [a11, a12, a13],
    #     [a12, a22, a23],
    #     [a13, a23, a33],
    # ])
    # A = sp.Matrix([ # only conventional lift and drag in the xz plane
    #     [a11, 0, a13],
    #     [0, 0, 0],
    #     [a13, 0, a33],
    # ])
    A = sp.Matrix([ # only conventional lift and drag in the xz plane
        [0, 0, 0],
        [0, a22, 0],
        [0, 0, a33],
    ])

    vIx, vIy, vIz = sp.symbols('vIx vIy vIz', real=True)
    v_symbols = vIx, vIy, vIz
    axx, axy, axz = sp.symbols('axx axy axz', real=True)
    ax_symbols = axx, axy, axz

    V = sp.symbols('V', real=True)
    v_I = sp.Matrix([vIx, vIy, vIz])
    ax_B = sp.Matrix([0, 0, -1])
    # ax_B = sp.Matrix([axx, axy, axz])
    fBz = sp.symbols('fBz', real=True)

    # # term1 = q^-1 ( A ( q v q^-1 ) ) q
    # qi = quat_conj(q)
    # term1 = sp.Matrix(rotate(qi, A[:, 0])) * v1
    # term1 += sp.Matrix(rotate(qi, A[:, 1])) * v2
    # term1 += sp.Matrix(rotate(qi, A[:, 2])) * v3
    # # w = sp.Matrix(rotate(q, v))
    # # Aw = A * w
    # #term1 = sp.Matrix(rotate(qinv, tuple(Aw)))

    # # term2 = s * q a q^-1
    # term2 = T * sp.Matrix(rotate(q, a_vec))


    # get_jacobian(term1 + term2, [q0, q1, q2, q3, T])

    # term1  = sp.Matrix(rotate(qi, A[:, 0])) * vIx
    # term1 += sp.Matrix(rotate(qi, A[:, 1])) * vIy
    # term1 += sp.Matrix(rotate(qi, A[:, 2])) * vIz
    # term2 = fBz*sp.Matrix(rotate(q, ax_B))

    term1  = sp.Matrix([0,0,0])
    for i in range(3):
        if (A[0, i]**2 + A[1, i]**2 + A[2, i]**2) != 0:
            term1 += sp.Matrix(sp.Quaternion.rotate_point(A[:, i], qi)) * v_I[i]
    term2 = fBz*sp.Matrix(sp.Quaternion.rotate_point(ax_B, q))

    expr_q = -V*term1 + term2

    Jq = get_jacobian(expr_q, [qw, qx, qy, qz, fBz])


    # Rotation vector
    dtx, dty, dtz = sp.symbols('dtx dty dtz', real=True)
    dTheta = sp.Matrix([dtx, dty, dtz])

    # Magnitude
    theta = sp.sqrt(dtx**2 + dty**2 + dtz**2)

    # Incremental quaternion, scalar first
    dq = sp.Matrix([
        1, dtx / 2, dty / 2, dtz / 2, fBz 
    ])

    # Jacobian dq / dTheta
    JTheta = dq.jacobian([dTheta, fBz])


    # total jacobian dChi / dTheta
    Jtotal = Jq @ JTheta


    if '--emit-c' in sys.argv:
        # Jtotal, substitutions = build_attitude_thrust_jacobian()
        symbols = (*q_symbols, *a_symbols, *ax_symbols, *v_symbols, V, fBz)
        c_expressions = (
            'w', 'x', 'y', 'z',
            *(f'a{row}{col}' for row in range(3) for col in range(row,3)),
            'axx', 'axy', 'axz',
            'vIx', 'vIy', 'vIz', 'V', 'fBz',
        )
        substitutions = dict(zip(symbols, c_expressions))
        emit_attitude_thrust_jacobian(
            Jtotal,
            substitutions,
            cse='--no-cse' not in sys.argv,
        )
        raise SystemExit(0)


    # ------------------------------------------------------------
    # Quaternion yaw constraint
    # ------------------------------------------------------------
    # from c-code in ZYX convention
    # e->angles.yaw   = atan2_approx((2.0f * (qp->wz - qp->xy)), (1.0f - 2.0f * (qp->xx + qp->zz)));
    n = 2 * (qw*qz - qx*qy)
    d = 1 - 2 * (qx**2 + qz**2)

    num, den = sp.symbols('num den', real=True)


    # Gradient of numerator and denominator wrt quaternion
    dn_dq = sp.Matrix([sp.diff(n, qi) for qi in q])
    dd_dq = sp.Matrix([sp.diff(d, qi) for qi in q])

    # Gradient of yaw
    #    d(atan2(num, den)) = ( den * d(num) - num * d(den) )  /  (den**2+num**2)
    # ignore denominator as it's always > 0 if q is unit quat

    common_den = (den**2+num**2)
    J_atan2 = sp.Matrix([sp.atan2(num, den)]).jacobian((num, den))
    g_q = (common_den * J_atan2) @ sp.Matrix([n, d]).jacobian(q)
    g_q = sp.simplify(g_q.subs(num, n).subs(den, d))
    # g_q = sp.simplify(d * dn_dq - n * dd_dq).T

    # Jacobian of q_plus with respect to dTheta at dTheta = 0
    Q = sp.Matrix([
        [-qx, -qy, -qz],
        [ qw, -qz,  qy],
        [ qz,  qw, -qx],
        [-qy,  qx,  qw]
    ])

    dq_dTheta = sp.Rational(1, 2) * Q

    C = sp.simplify(g_q @ dq_dTheta)

    C_unit = sp.Matrix([
        sp.factor(expr.subs(
            qw**2,
            1 - qx**2 - qy**2 - qz**2
        ))
        for expr in C
    ])

    C_unit = sp.simplify(C_unit)

    sp.pprint(C_unit)


    C_unit_2 = sp.Matrix([
        sp.expand(c).subs(
            qw**2,
            1 - qz**2 - qx**2 - qy**2
        )
        for c in C
    ])

    C_unit_2 = sp.simplify(C_unit_2)

    sp.pprint(C_unit_2)



    # phi, theta, psi = sp.symbols('phi theta psi', real=True)
    # Rpsi = build_rotmat(2, psi)
    # Rtheta = build_rotmat(1, theta)
    # Rphi = build_rotmat(0, phi)

    # R = Rpsi @ Rphi @ Rtheta
    # Ri = R.T

    # expr = - R @ A @ R.T @ vI + T * R @ aB

    # # J = get_jacobian(expr, [phi, theta, T], cse=False)


    # # reduced form: find euler angles 
    # # phi, theta, psi = sp.symbols('phi theta psi', real=True)

    # # Rpsi = build_rotmat(2, psi)
    # # Rtheta = build_rotmat(1, theta)
    # # Rphi = build_rotmat(0, phi)

    # asq = sp.symbols('asq', real=True)

    # G = 9.81
    # # lift = -asq*sp.sin(theta) * G
    # # T = -sp.cos(theta) * G
    # T = sp.symbols('T', real=True)
    # lift = sp.Function('lift')

    # Rterm1 = Rpsi * Rphi * ( sp.Matrix([0, 0, lift(theta)]) + Rtheta * sp.Matrix([0, 0, T]) )

    # Je = get_jacobian(Rterm1, [phi, theta, T])




    # reduced form: find euler angles 
    # phi, theta, psi = sp.symbols('phi theta psi', real=True)

    # Rpsi = build_rotmat(2, psi)
    # Rtheta = build_rotmat(1, theta)
    # Rphi = build_rotmat(0, phi)
    # Ripsi = build_rotmat(2, -psi)
    # Ritheta = build_rotmat(1, -theta)
    # Riphi = build_rotmat(0, -phi)

    # R = Rpsi * Rphi * Rtheta
    # Ri = Ritheta * Riphi * Ripsi

    # F = R * ( A * Ri * sp.Matrix(v)  +  T * sp.Matrix(a_vec))

    # JR = get_jacobian(F, [phi, theta, T])
