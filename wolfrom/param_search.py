"""
Port of the MATLAB code for the brute-force search on gear parameters.
This SHOULD yield the same result as the MATLAB version.
"""


import numpy as np
import matplotlib.pyplot as plt

def e_calc(Zs, Zp1, Zr1, Zp2, Zr2, Xs, Xp1, Xp2, Xr1, Xr2, ma, mc, alpha, mu):
    """
    Calculate the back drive efficiency given the gear parameters.
    
    Parameters:
      Zs, Zp1, Zr1, Zp2, Zr2 : float
          Number of teeth for sun, planet1, ring1, planet2, ring2 respectively.
      Xs, Xp1, Xp2, Xr1, Xr2 : float
          Profile shift coefficients.
      ma, mc : float
          Module values.
      alpha : float
          Base pressure angle in radians.
      mu : float
          Friction coefficient.
    
    Returns:
      backEfficiency : float
          The calculated back drive efficiency.
    """
    I1 = Zr1 / Zs
    I2 = Zr1 * Zp2 / (Zp1 * Zr2)
    ra = (Zs * ma + Zp1 * ma) / 2.0

    db_a1 = ma * Zs * np.cos(alpha)
    db_a2 = ma * Zp1 * np.cos(alpha)
    db_b1 = ma * Zp1 * np.cos(alpha)
    db_b2 = ma * Zr1 * np.cos(alpha)
    db_c1 = mc * Zp2 * np.cos(alpha)
    db_c2 = mc * Zr2 * np.cos(alpha)
    
    # These variables are computed the same way as db_a1 etc.
    db_s = ma * Zs * np.cos(alpha)
    db_p1 = ma * Zp1 * np.cos(alpha)
    db_r1 = ma * Zr1 * np.cos(alpha)
    db_p2 = mc * Zp2 * np.cos(alpha)
    db_r2 = mc * Zr2 * np.cos(alpha)
    
    alpha_wa = np.arccos( ma * (Zs + Zp1) * np.cos(alpha) / (2 * ra) )
    alpha_wb = np.arccos( ma * (-Zp1 + Zr1) * np.cos(alpha) / (2 * ra) )
    alpha_wc = np.arccos( mc * (-Zp2 + Zr2) * np.cos(alpha) / (2 * ra) )
    
    ya = ((Zs + Zp1) / 2.0) * ((np.cos(alpha) / np.cos(alpha_wa)) - 1)
    
    das  = ma * Zs + 2 * ma * (1 + ya - Xp1)
    dap1 = ma * Zp1 + 2 * ma * (1 + min(ya - Xs, Xp1))
    dar1 = ma * Zr1 - 2 * ma * (1 - Xr1)
    dap2 = mc * Zp2 + 2 * mc * (1 + Xp2)
    dar2 = mc * Zr2 - 2 * mc * (1 - Xr2)
    
    alpha_aa1 = np.arccos(db_a1 / das)
    alpha_aa2 = np.arccos(db_a2 / dap1)
    alpha_ab1 = np.arccos(db_b1 / dap1)
    alpha_ab2 = np.arccos(db_b2 / dar1)
    alpha_ac1 = np.arccos(db_c1 / dap2)
    alpha_ac2 = np.arccos(db_c2 / dar2)
    
    ea1 = (Zp1 / (2 * np.pi)) * (np.tan(alpha_aa2) - np.tan(alpha_wa))
    ea2 = (Zs / (2 * np.pi)) * (np.tan(alpha_aa1) - np.tan(alpha_wa))
    eb1 = -(Zr1 / (2 * np.pi)) * (np.tan(alpha_ab2) - np.tan(alpha_wb))
    eb2 = (Zp1 / (2 * np.pi)) * (np.tan(alpha_ab1) - np.tan(alpha_wb))
    ec1 = -(Zr2 / (2 * np.pi)) * (np.tan(alpha_ac2) - np.tan(alpha_wc))
    ec2 = (Zp2 / (2 * np.pi)) * (np.tan(alpha_ac1) - np.tan(alpha_wc))
    
    ea = ea1**2 + ea2**2 - ea1 - ea2 + 1
    eb = eb1**2 + eb2**2 - eb1 - eb2 + 1
    ec = ec1**2 + ec2**2 - ec1 - ec2 + 1
    
    Ea = 1 - mu * np.pi * (1 / Zs + 1 / Zp1) * ea
    Eb = 1 - mu * np.pi * (1 / Zp1 - 1 / Zr1) * eb
    Ec = 1 - mu * np.pi * (1 / Zp2 - 1 / Zr2) * ec
    
    backEfficiency = (1 + I1) * Ea * (Eb * Ec - I2) / ( Ec * (Ea * Eb + I1) * (1 - I2) )
    return backEfficiency

def brute_force_search_Z():
    """
    Run a parameter sweep (brute-force search) on the number of teeth (Z values)
    and return efficiency values versus percent changes.
    """
    # Default parameter values from the base model
    ma = 0.8e-3
    mc = 0.8467e-3
    alpha = (20 / 180) * np.pi
    mu = 0.04
    Zs_default   = 12
    Zp1_default  = 39
    Zr1_default  = 90
    Zp2_default  = 32
    Zr2_default  = 81
    Xs_default   = 0.476
    Xp1_default  = 0.762
    Xp2_default  = 0.536
    Xr1_default  = 2
    Xr2_default  = 1.21

    # Sweep on Zr1 (ring1): 27 values from 66 to 92
    Zr1_range = np.arange(66, 66 + 27)
    E_Zr1 = np.array([e_calc(Zs_default, Zp1_default, Zr1, Zp2_default, Zr2_default,
                              Xs_default, Xp1_default, Xp2_default, Xr1_default, Xr2_default,
                              ma, mc, alpha, mu)
                        for Zr1 in Zr1_range])
    
    # Sweep on Zs (sun): 12 values from 10 to 21
    Zs_range = np.arange(10, 10 + 12)
    E_Zs = np.array([e_calc(Zs, Zp1_default, Zr1_default, Zp2_default, Zr2_default,
                             Xs_default, Xp1_default, Xp2_default, Xr1_default, Xr2_default,
                             ma, mc, alpha, mu)
                     for Zs in Zs_range])
    
    # Sweep on Zp1 (planet1): 13 values from 38 to 50
    Zp1_range = np.arange(38, 38 + 13)
    E_Zp1 = np.array([e_calc(Zs_default, Zp1, Zr1_default, Zp2_default, Zr2_default,
                              Xs_default, Xp1_default, Xp2_default, Xr1_default, Xr2_default,
                              ma, mc, alpha, mu)
                        for Zp1 in Zp1_range])
    
    # Sweep on Zp2 (planet2): 5 values from 30 to 34
    Zp2_range = np.arange(30, 30 + 5)
    E_Zp2 = np.array([e_calc(Zs_default, Zp1_default, Zr1_default, Zp2, Zr2_default,
                              Xs_default, Xp1_default, Xp2_default, Xr1_default, Xr2_default,
                              ma, mc, alpha, mu)
                        for Zp2 in Zp2_range])
    
    # Sweep on Zr2 (ring2): 8 values from 76 to 83
    Zr2_range = np.arange(76, 76 + 8)
    E_Zr2 = np.array([e_calc(Zs_default, Zp1_default, Zr1_default, Zp2_default, Zr2,
                              Xs_default, Xp1_default, Xp2_default, Xr1_default, Xr2_default,
                              ma, mc, alpha, mu)
                        for Zr2 in Zr2_range])
    
    # Create corresponding percent change arrays (as in the MATLAB code)
    pct_Zr1 = np.linspace(-2600 / (2 * 90), 2600 / (2 * 90), 27)
    pct_Zr2 = np.linspace(-350 / (2 * 81), 350 / (2 * 81), 8)
    pct_Zp1 = np.linspace(-600 / (2 * 39), 600 / (2 * 39), 13)
    pct_Zp2 = np.linspace(-200 / (2 * 32), 200 / (2 * 32), 5)
    pct_Zs  = np.linspace(-550 / (2 * 12), 550 / (2 * 12), 12)
    
    return {
        'pct_Zs': pct_Zs, 'E_Zs': E_Zs,
        'pct_Zr1': pct_Zr1, 'E_Zr1': E_Zr1,
        'pct_Zp1': pct_Zp1, 'E_Zp1': E_Zp1,
        'pct_Zp2': pct_Zp2, 'E_Zp2': E_Zp2,
        'pct_Zr2': pct_Zr2, 'E_Zr2': E_Zr2
    }

def brute_force_search_X():
    """
    Run a parameter sweep on the profile shift coefficients (X values)
    and return efficiency values versus percent changes.
    """
    # Reset default Z values and other parameters
    ma = 0.8e-3
    mc = 0.8467e-3
    alpha = (20 / 180) * np.pi
    mu = 0.04
    Zs_default   = 12
    Zp1_default  = 39
    Zr1_default  = 90
    Zp2_default  = 32
    Zr2_default  = 81
    Xs_default   = 0.476
    Xp1_default  = 0.762
    Xp2_default  = 0.536
    Xr1_default  = 2
    Xr2_default  = 1.21

    # Sweep on Xs: 43 values, Xs from 0 to 42*0.05
    Xs_vals = np.arange(43) * 0.05
    E_xs = np.array([e_calc(Zs_default, Zp1_default, Zr1_default, Zp2_default, Zr2_default,
                             Xs, Xp1_default, Xp2_default, Xr1_default, Xr2_default,
                             ma, mc, alpha, mu)
                     for Xs in Xs_vals])
    
    # Sweep on Xp1: 27 values
    Xp1_vals = np.arange(27) * 0.05
    E_xp1 = np.array([e_calc(Zs_default, Zp1_default, Zr1_default, Zp2_default, Zr2_default,
                              Xs_default, Xp1, Xp2_default, Xr1_default, Xr2_default,
                              ma, mc, alpha, mu)
                      for Xp1 in Xp1_vals])
    
    # Sweep on Xp2: 201 values
    Xp2_vals = np.arange(201) * 0.05
    E_xp2 = np.array([e_calc(Zs_default, Zp1_default, Zr1_default, Zp2_default, Zr2_default,
                              Xs_default, Xp1_default, Xp2, Xr1_default, Xr2_default,
                              ma, mc, alpha, mu)
                      for Xp2 in Xp2_vals])
    
    # Sweep on Xr1: 181 values
    Xr1_vals = np.arange(181) * 0.05
    E_xr1 = np.array([e_calc(Zs_default, Zp1_default, Zr1_default, Zp2_default, Zr2_default,
                              Xs_default, Xp1_default, Xp2_default, Xr1, Xr2_default,
                              ma, mc, alpha, mu)
                      for Xr1 in Xr1_vals])
    
    # Sweep on Xr2: 141 values
    Xr2_vals = np.arange(141) * 0.05
    E_xr2 = np.array([e_calc(Zs_default, Zp1_default, Zr1_default, Zp2_default, Zr2_default,
                              Xs_default, Xp1_default, Xp2_default, Xr1_default, Xr2,
                              ma, mc, alpha, mu)
                      for Xr2 in Xr2_vals])
    
    # Scale efficiencies as in the MATLAB code (multiply by 100/0.8562)
    scale_factor = 100 / 0.8562
    E_xs  *= scale_factor
    E_xp1 *= scale_factor
    E_xp2 *= scale_factor
    E_xr1 *= scale_factor
    E_xr2 *= scale_factor
    
    # Define percent change arrays for plotting
    pct_Xr1 = np.linspace(0, 900 / 2, 181)
    pct_Xr2 = np.linspace(0, 700 / 1.21, 141)
    pct_Xp1 = np.linspace(0, 130 / 0.762, 27)
    pct_Xp2 = np.linspace(0, 1000 / 0.536, 201)
    pct_Xs  = np.linspace(0, 210 / 0.476, 43)
    
    return {
        'pct_Xs': pct_Xs, 'E_xs': E_xs,
        'pct_Xr1': pct_Xr1, 'E_xr1': E_xr1,
        'pct_Xp1': pct_Xp1, 'E_xp1': E_xp1,
        'pct_Xp2': pct_Xp2, 'E_xp2': E_xp2,
        'pct_Xr2': pct_Xr2, 'E_xr2': E_xr2
    }

if __name__ == '__main__':
    # Run the brute-force search on gear tooth numbers (Z values)
    results_Z = brute_force_search_Z()
    plt.figure()
    plt.plot(results_Z['pct_Zs'], results_Z['E_Zs'], linewidth=2, label='sun')
    plt.plot(results_Z['pct_Zr1'], results_Z['E_Zr1'], linewidth=2, label='ring1')
    plt.plot(results_Z['pct_Zr2'], results_Z['E_Zr2'], linewidth=2, label='ring2')
    plt.plot(results_Z['pct_Zp1'], results_Z['E_Zp1'], linewidth=2, label='planet1')
    plt.plot(results_Z['pct_Zp2'], results_Z['E_Zp2'], linewidth=2, label='planet2')
    plt.legend()
    plt.grid(True, which='both', linestyle='--')
    plt.title('Efficiency vs Number of Teeth')
    plt.xlabel('Percent change in Number of Teeth')
    plt.ylabel('Percent response in Efficiency')
    
    # Run the brute-force search on profile shift coefficients (X values)
    results_X = brute_force_search_X()
    plt.figure()
    plt.plot(results_X['pct_Xs'], results_X['E_xs'], linewidth=2, label='sun')
    plt.plot(results_X['pct_Xr1'], results_X['E_xr1'], linewidth=2, label='ring1')
    plt.plot(results_X['pct_Xr2'], results_X['E_xr2'], linewidth=2, label='ring2')
    plt.plot(results_X['pct_Xp1'], results_X['E_xp1'], linewidth=2, label='planet1')
    plt.plot(results_X['pct_Xp2'], results_X['E_xp2'], linewidth=2, label='planet2')
    plt.legend()
    plt.grid(True, which='both', linestyle='--')
    plt.title('Efficiency vs Profile Shift')
    plt.xlabel('Percent change in Profile Shift Coefficient')
    plt.ylabel('Percent response in Efficiency')
    
    plt.show()
