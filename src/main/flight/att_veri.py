import numpy as np

def getAttSpNedFromAccSpNed(accSpNed):
    Psi = 0
    cPsi = np.cos(Psi)
    sPsi = np.sin(Psi)

    aSp_min_g = accSpNed - np.array([0, 0, 9.80665])
    v = np.array([
        cPsi * aSp_min_g[0] + sPsi * aSp_min_g[1],
        -sPsi * aSp_min_g[0] + cPsi * aSp_min_g[1],
        aSp_min_g[2]
    ])

    ax = np.zeros((3,), dtype=float)
    vx2 = v[0]*v[0]
    vy2 = v[1]*v[1]
    XYnorm = np.sqrt(vx2 + vy2)
    if ( XYnorm < 1e-4 ):
        ax[0] = (+1 if v[1] >= 0 else -1) if (vy2 >= vx2) else 0
        ax[1] = (-1 if v[0] >= 0 else +1) if (vy2 <  vx2) else 0
    else:
        if (vy2 > vx2):
            ax[0]  =  v[1]  /  XYnorm;
            ax[1]  =  - (v[0] * ax[0]) / v[1];
        else:
            ax[1]  =  - v[0]  /  XYnorm;
            ax[0]  =  - (v[1] * ax[1]) / v[0];

    alpha = np.arctan2( XYnorm, -v[2])
    alpha = np.clip(alpha, 0, np.pi)


