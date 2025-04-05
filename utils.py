import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.transforms import Affine2D
import time
from matplotlib.offsetbox import AnchoredText 
import mplcursors 
import os
import json

import itertools

from tqdm import tqdm

config_path = os.path.join(os.path.dirname(__file__), 'wolfrom', "config.json")
with open(config_path, "r") as f:
    config = json.load(f)

MODULE = config["MODULE"]
MU = config["MU"]
MC = config.get("MC", MODULE)
ALPHA = np.radians(config.get("ALPHA", 20))
XR1 = config.get("XR1", 0.0)
N_PLANETS = config["N_PLANETS"]
TARGET_GEAR_RATIO = 100
RATIO_TOLERANCE = 10
target_z_r1 = 90

'''
Function 'validate parameters'
takes: in_vals (parameters to sweep) [7,n] - [z_s, z_p2, z_r2, x_s, x_p1, x_p2, x_r2]
       target_z_r1 - target gears for ring 1 ()

returns: subset of input values that pass tolerance check: z_s, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2
         calculated values: I1, I2, gr_s, ratios
'''
def validate_parameters(in_vals):

    # Calculate all possible combinations of gear ratios
    z_sh, z_p2, z_r2 = np.meshgrid(
        *in_vals[:3], indexing='ij'
    )
    
    z_s = z_sh * 2 # enforce even values for z_s

    z_r1 = target_z_r1
    z_p1 = (z_r1 - z_s)/2
    non_integers = z_p1[np.mod(z_p1, 1) != 0]
    if len(non_integers) > 0:
        print('Invalid z_s values (not divisble by 2)')
        return 0

    I1 = z_r1 / z_s
    I2 = (z_r1 * z_p2) / (z_p1 * z_r2)
    gr_s = (1 - I2) / (1 + I1)
    ratios = 1 / gr_s

    # only keep valid combinations (ratios w.in tolerance)
    valid_combinations = np.logical_and(gr_s > 0, np.abs(TARGET_GEAR_RATIO - ratios) <= RATIO_TOLERANCE)
    filt_z_s = np.unique(z_s[valid_combinations])
    filt_z_p2 = np.unique(z_p2[valid_combinations])
    filt_z_r2 = np.unique(z_r2[valid_combinations])

    z_s, z_p2, z_r2, xs, xp1, xp2, xr2 = np.meshgrid(
        filt_z_s, filt_z_p2, filt_z_r2, *in_vals[3:], indexing='ij'
    )

    # flatten
    z_s = z_s.flatten()
    z_p2 = z_p2.flatten()
    z_r2 = z_r2.flatten()
    xs = xs.flatten()
    xp1 = xp1.flatten()
    xp2 = xp2.flatten()
    xr2 = xr2.flatten()

    z_p1 = (z_r1 - z_s)/2

    I1 = z_r1 / z_s
    I2 = (z_r1 * z_p2) / (z_p1 * z_r2)
    gr_s = (1 - I2) / (1 + I1)
    ratios = 1 / gr_s

    
    total_combinations = z_s.size

    valid_combinations = np.logical_and(np.logical_and(gr_s > 0, np.abs(TARGET_GEAR_RATIO - ratios) <= RATIO_TOLERANCE), z_s + 2 * z_p1 == z_r1)

    # only keep valid combinations (ratios w.in tolerance)
    tol = 0.1 # let's start with 0.1, the allowable tolerance in table III is 0.09 as a reference.

    # Stage 2 constraint Ring 2 + Planet 2
    # Derived equation: (z_r2 - z_p2) = (MODULE/MC) * (z_s + z_p1 + 2 * (xs + xp1)) - 2 * (xr2 - xp2)
    ls_stage2 = z_r2 - z_p2 # left side of eq
    rs_stage2 = (MODULE / MC) * (z_s + z_p1 + 2 * (xs + xp1)) - 2 * (xr2 - xp2) # right side of eq

    # TODO: check if this is the right way to include x values, right now im going off the equations from this page
    # https://www.tec-science.com/mechanical-power-transmission/involute-gear/profile-shift/#:~:text=Profile%20shift%20coefficient,For%20the%20corresponding%20diameters%20applies:

    stage_2_constraint = np.abs(ls_stage2 - rs_stage2) < tol # check with tolerance 

    # check it again with existing valid combos...
    valid_combinations = np.logical_and(valid_combinations, stage_2_constraint)

    # z_r1 = z_r1[valid_combinations]
    z_s, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2 = z_s[valid_combinations], z_p1[valid_combinations], z_p2[valid_combinations], z_r2[valid_combinations], xs[valid_combinations], xp1[valid_combinations], xp2[valid_combinations], xr2[valid_combinations]
    I1, I2, gr_s, ratios = I1[valid_combinations], I2[valid_combinations], gr_s[valid_combinations], ratios[valid_combinations]

    assert z_s.shape == z_p1.shape == z_p2.shape == z_r2.shape == xs.shape == xp1.shape == xp2.shape == xr2.shape == ratios.shape

    z_sh = z_s / 2 # convert back to z_s / 2 for output

    return z_sh, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2, I1, I2, gr_s, ratios

'''
Function 'efficiency_calc'
takes:
gives: fwd efficiency
       bwd eddiciency
'''
def efficiency_calc(z_sh, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2, I1, I2, gr_s, ratios):
    z_s = z_sh * 2 # enforce even values for z_s
    ra = (z_s * MODULE + z_p1 * MODULE) / 2.0

    # Basic circle diameters
    db_a1 = MODULE * z_s * np.cos(ALPHA)
    db_a2 = MODULE * z_p1 * np.cos(ALPHA)
    db_b1 = MODULE * z_p1 * np.cos(ALPHA)
    db_b2 = MODULE * target_z_r1 * np.cos(ALPHA)
    db_c1 = MC * z_p2 * np.cos(ALPHA)
    db_c2 = MC * z_r2 * np.cos(ALPHA)

    # Working pressure angles (modified formula)
    alpha_wa = np.arccos(MODULE * (z_s + z_p1) * np.cos(ALPHA) / (2 * ra))
    alpha_wb = np.arccos(MODULE * (-z_p1 + target_z_r1) * np.cos(ALPHA) / (2 * ra)) # why is this constant??
    val = MC * (-z_p2 + z_r2) * np.cos(ALPHA) / (2 * ra)
    alpha_wc = np.arccos(np.clip(val, -1, 1))
    # Center distance modification coefficient
    ya = ((z_s + z_p1) / 2.0) * ((np.cos(ALPHA) / np.cos(alpha_wa)) - 1)
    # Tip circle diameters
    das  = MODULE * z_s + 2 * MODULE * (1 + ya - xp1)
    dap1 = MODULE * z_p1 + 2 * MODULE * (1 + np.minimum(ya - xs, xp1))
    dar1 = MODULE * target_z_r1 - 2 * MODULE * (1 - XR1)
    dap2 = MC * z_p2 + 2 * MC * (1 + xp2)
    dar2 = MC * z_r2 - 2 * MC * (1 - xr2)
    # Tip pressure angles
    alpha_aa1 = np.arccos(db_a1 / das)
    alpha_aa2 = np.arccos(db_a2 / dap1)
    alpha_ab1 = np.arccos(db_b1 / dap1)
    alpha_ab2 = np.arccos(db_b2 / dar1)
    alpha_ac1 = np.arccos(db_c1 / dap2)
    val2 = db_c2 / dar2
    alpha_ac2 = np.arccos(np.clip(val2, -1, 1))
    # Approach and recess contact ratios
    ea1 = (z_p1 / (2 * np.pi)) * (np.tan(alpha_aa2) - np.tan(alpha_wa))
    ea2 = (z_s / (2 * np.pi)) * (np.tan(alpha_aa1) - np.tan(alpha_wa))
    eb1 = -(target_z_r1 / (2 * np.pi)) * (np.tan(alpha_ab2) - np.tan(alpha_wb))
    eb2 = (z_p1 / (2 * np.pi)) * (np.tan(alpha_ab1) - np.tan(alpha_wb))
    ec1 = -(z_r2 / (2 * np.pi)) * (np.tan(alpha_ac2) - np.tan(alpha_wc))
    ec2 = (z_p2 / (2 * np.pi)) * (np.tan(alpha_ac1) - np.tan(alpha_wc))
    ea = ea1**2 + ea2**2 - ea1 - ea2 + 1
    eb = eb1**2 + eb2**2 - eb1 - eb2 + 1
    ec = ec1**2 + ec2**2 - ec1 - ec2 + 1
    # Basic driving efficiencies
    Ea_val = 1 - MU* np.pi * (1 / z_s + 1 / z_p1) * ea
    Eb_val = 1 - MU* np.pi * (1 / z_p1 - 1 / target_z_r1) * eb
    Ec_val = 1 - MU* np.pi * (1 / z_p2 - 1 / z_r2) * ec
    # Forward driving efficiency 
    eta_fwd = (1 + Ea_val * Eb_val * I1) * (1 - I2) / ((1 + I1) * (1 - Eb_val * Ec_val * I2))
    # Backward driving efficiency
    def eta_bwd1(I1, I2, Ea_val, Eb_val, Ec_val):
        return (1 + I1) * Ea_val * (Eb_val * Ec_val - I2) / (Ec_val * (Ea_val * Eb_val + I1) * (1 - I2))
    def eta_bwd2(I1, I2, Ea_val, Eb_val, Ec_val):
        return (1 + I1)*Ea_val*(1-Eb_val*Ec_val*I2)/((Ea_val+Eb_val*I1)*(1-I2))
    eta_bwd = np.where(I2 > 1, eta_bwd2(I1, I2, Ea_val, Eb_val, Ec_val), eta_bwd1(I1, I2, Ea_val, Eb_val, Ec_val))
    # eta_bwd = (1 + I1) * Ea_val * (Eb_val * Ec_val - I2) / (Ec_val * (Ea_val * Eb_val + I1) * (1 - I2))

    # filter high eta_bwd
    eta_bwd = np.where(eta_bwd > 1, 0, eta_bwd)
    eta_bwd = np.where(eta_bwd < 0, 0, eta_bwd)

    return eta_fwd, eta_bwd



'''
Function 'score vals'
takes:  in_vals (parameters to sweep) [7,1] list of  - [z_sh, z_p2, z_r2, x_s, x_p1, x_p2, x_r2]
        target_z_r1 - target gears for ring 1 ()
        add_gear - boolean whether to also add in the values from Matsuki 2019 paper
gives: score [scalar] - number of valid combinations
NOTE - for scores to be transferable, give the same size vectors across different in_vals

'''

def score_vals(in_vals, target_z_r1 = 90, add_gear = True):

    # 2019 Matsuki 'Table 3' values
    paper_vals = [6, 32, 81, .476, .762, .536, 1.2] # NOTE - 6 = 12 (z_s) / 2.

    if add_gear:
        for i in range(len(in_vals)):
            in_vals[i] = np.hstack([in_vals[i], paper_vals[i]])

    # validated_vals = z_s, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2, I1, I2, gr_s, ratios
    validated_vals = validate_parameters(in_vals)

    eta_fwd, eta_bwd = efficiency_calc(*validated_vals)

    # composite_score = ratios * eta_fwd * eta_bwd # possibly useful in future ?
    bd_indices = np.where(eta_bwd > 0.3)

    score = len(validated_vals[0][bd_indices])
    return score

'''
Function 'param_to_list'
takes:  params - dictionary from Ax.dev detailing parameters
gives: parm_list [7,1] list for use in score_vals  - [z_sh, z_p2, z_r2, x_s, x_p1, x_p2, x_r2]

'''
def param_to_list(param):
    each_val_len = len(param) // 7 # 7 parameters
    param_list = []
    for i in range(7):
        # if i == 0: # note first (z_s) inputs are actually z_s / 2 to enforce even values.
        #     param_list.append(np.array(list(param.values())[i*each_val_len:(i+1)*each_val_len])*2)
        # else:
        # NOTE - as of 4/5, we don't ever think of 'z_s', we use 'z_sh' (half value)
        param_list.append(np.array(list(param.values())[i*each_val_len:(i+1)*each_val_len]))
    return param_list


def list_to_param(param_list, naming = ['z_sh', 'z_p2', 'z_r2', 'x_s', 'x_p1', 'x_p2', 'x_r2'], vals_per = 3):
    """
    Function 'list_to_param'
    takes: param_list [7,1] list for use in score_vals - [z_s, z_p2, z_r2, x_s, x_p1, x_p2, x_r2]
    gives: params - dictionary from Ax.dev detailing parameters
    """

    param = {}

    for ind in range(len(param_list)):
        if naming[ind][0] == 'z':
            if len(param_list[ind]) == 1:
                param[naming[ind]] = int(param_list[ind][0])
                continue
            else:
                param[naming[ind]] = int(param_list[ind][0])
                for ind2 in range(1, vals_per):
                    param[naming[ind] + '_' + str(ind2)] = int(param_list[ind][ind2])
        elif naming[ind][0] == 'x':
            if len(param_list[ind]) == 1:
                param[naming[ind]] = float(param_list[ind][0])
                continue
            else:
                param[naming[ind]] = float(param_list[ind][0])
                for ind2 in range(1, vals_per):
                    param[naming[ind] + '_' + str(ind2)] = float(param_list[ind][ind2])

    return param