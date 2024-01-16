import pulp as pl

prob = pl.LpProblem("Motor probing matrix", pl.LpMinimize)
M1_m = pl.LpVariable("")