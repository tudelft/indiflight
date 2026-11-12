"""Library code for tailsitter model identification from indiflight logs.

Modules
-------
data        log loading, cropping/auto-ranging, filtered signal bundle (FlightData)
actuation   ActuationModel interface, physical (TailsitterPhi-like) model,
            onboard-learner model, save/load
global_phi  library port of tailsitterGlobalPhi.py (full actuation+aero fit)
offline_rls library port of the moment RLS models of rlsPrototypingTailsitter.py
aero_fit    phi-theory aero regression on the actuation-model residual
"""
