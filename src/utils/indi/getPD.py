from sage.all import *

# Work in progress
# hopefully at somepoint automatically tunes PD based on effectiveness and act time constant

(s,P,D,tau,t) = var('s P D tau t')
assume(P,'real')
assume(D,'real')
assume(tau,'real')
assume(P > 0)
assume(D > 0)
assume(tau > 0)
assume(4*D*tau - P > 0)
assume(4*D*P*tau + 4*D*tau - P > 0)
#assume(4*D*P**2*tau + 4*D*P*tau - P**2 > 0)
assume(t,'real')

# system
G = 1 / (1 + s*tau)

# inner loop
rate_OL = D/P* G * 1/s
rate = rate_OL / (1 + rate_OL)

# outer loop
angle_OL = P*rate
angle = angle_OL / (1 + angle_OL)

# response
res = inverse_laplace(angle, s, t).simplify()
