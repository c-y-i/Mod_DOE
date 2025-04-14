"""
Utility functions for the Wolfrom gear design optimization process.
"""

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

MA = config["MODULE"]
MU = config["MU"]
MC = config.get("MC", MA)
ALPHA = np.radians(config.get("ALPHA", 20))
XR1 = config.get("XR1", 0.0)
N_PLANETS = config["N_PLANETS"]
TARGET_GEAR_RATIO = 100
RATIO_TOLERANCE = 10
TARG_zr1 = 90
TARG_xr1 = 0


'''
Function 'validate parameters'
takes: in_vals (parameters to sweep) [5,n] - [z_sh, z_r2, xs, xr2, Cl]
       offsets [2,n] - p1_offset, p2_offset (optional)
       target_z_r1 - target gears for ring 1 ()
       target_xr1 - target profile shift for ring 1 ()

returns: design_vals: [z_sh, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2, carrier]
         calculated values: I1, I2, gr_s, ratios
'''
def validate_parameters(in_vals, offsets = None, target_z_r1 = TARG_zr1, target_xr1 = TARG_xr1):

    # Calculate all possible combinations of gear ratios
    if offsets is not None:
        in_vals.append(offsets[0])
        in_vals.append(offsets[1])
        z_sh, z_r2, xs, xr2, Cl, p1_offset, p2_offset = np.meshgrid(
            *in_vals, indexing='ij'
        )
        p1_offset = p1_offset.flatten()
        p2_offset = p2_offset.flatten()
    else:   
        z_sh, z_r2, xs, xr2, Cl = np.meshgrid(
            *in_vals, indexing='ij'
        )
        p1_offset, p2_offset = 0, 0

    Cl = Cl.flatten()

    Cl_1 = Cl_2 = Cl_3 = Cl

    z_sh = z_sh.flatten()
    z_r2 = z_r2.flatten()
    xs = xs.flatten()
    xr2 = xr2.flatten()

    z_s = z_sh * 2 # enforce even values for z_s

    z_r1 = target_z_r1
    xr1 = target_xr1 

    eqn1 = (z_r1+2*xr1-z_s-2*xs)/2 - (Cl_1 + Cl_2)/MA
    # NOTE - the following is a restrictive design choice, mostly for convenience in CAD
    # and for reliably reconciling this equation.
    z_p1 = np.floor(eqn1) + p1_offset
    xp1 = (eqn1 - z_p1)/2 # limits xp1 between 0 and 0.5

    eqn2 = MA/MC*((z_p1+2*xp1) - (z_r1+2*xr1)) + (z_r2 + 2*xr2) + 2/MC*(Cl_2 - Cl_3)
    # NOTE - the following is a restrictive design choice, mostly for convenience in CAD
    # and for reliably reconciling this equation.
    z_p2 = np.floor(eqn2) + p2_offset
    xp2 = (eqn2 - z_p2)/2 # limits xp2 between 0 and 0.5

    # solve r_a / r_b / r_c
    r_a = MA * (z_s + z_p1)/2 + MA *(xs + xp1) + Cl

    I1 = z_r1 / z_s
    I2 = (z_r1 * z_p2) / (z_p1 * z_r2)
    gr_s = np.abs((1 - I2) / (1 + I1))
    ratios = 1 / gr_s

    # TODO - add back in target gear ratio (?) - would need to take into account +/- values.
    # example: valid_combinations = np.abs(TARGET_GEAR_RATIO - np.abs(ratios))
    
    total_combinations = z_s.size

    # print(f"Total combinations: {total_combinations}, Valid combinations: {np.sum(valid_combinations)}, Stage 2 constraint: {np.sum(stage_2_constraint)}")

    # # z_r1 = z_r1[valid_combinations]
    # z_s, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2 = z_s[valid_combinations], z_p1[valid_combinations], z_p2[valid_combinations], z_r2[valid_combinations], xs[valid_combinations], xp1[valid_combinations], xp2[valid_combinations], xr2[valid_combinations]
    # I1, I2, gr_s, ratios = I1[valid_combinations], I2[valid_combinations], gr_s[valid_combinations], ratios[valid_combinations]

    # print(f'Shapes: {z_s.shape}, {z_p1.shape}, {z_p2.shape}, {z_r2.shape}, {xs.shape}, {xp1.shape}, {xp2.shape}, {xr2.shape}, {ratios.shape}')
    assert z_s.shape == z_p1.shape == z_p2.shape == z_r2.shape == xs.shape == xp1.shape == xp2.shape == xr2.shape == ratios.shape

    z_sh = z_s / 2 # convert back to z_s / 2 for output

    design_vals = [z_sh, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2, r_a]

    return design_vals, I1, I2, gr_s, ratios

'''
Function 'efficiency_calc'
takes: design_vals [8,n] - [z_sh, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2, r_a]
gives: fwd efficiency
       bwd eddiciency
'''
def efficiency_calc(design_vals, target_z_r1 = TARG_zr1, mu = MU):
    z_sh, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2, ra = design_vals
    z_s = z_sh * 2 # enforce even values for z_s

    z_r1 = TARG_zr1

    I1 = z_r1 / z_s
    I2 = (z_r1 * z_p2) / (z_p1 * z_r2)

    # Basic circle diameters
    db_a1 = MA * z_s * np.cos(ALPHA)
    db_a2 = MA * z_p1 * np.cos(ALPHA)
    db_b1 = MA * z_p1 * np.cos(ALPHA)
    db_b2 = MA * target_z_r1 * np.cos(ALPHA)
    db_c1 = MC * z_p2 * np.cos(ALPHA)
    db_c2 = MC * z_r2 * np.cos(ALPHA)

    # Working pressure angles (modified formula)
    alpha_wa = np.arccos(MA * (z_s + z_p1) * np.cos(ALPHA) / (2 * ra))
    alpha_wb = np.arccos(MA * (-z_p1 + target_z_r1) * np.cos(ALPHA) / (2 * ra)) # why is this constant??
    val = MC * (-z_p2 + z_r2) * np.cos(ALPHA) / (2 * ra)
    alpha_wc = np.arccos(np.clip(val, -1, 1))

    # Center distance modification coefficient
    ya = ((z_s + z_p1) / 2.0) * ((np.cos(ALPHA) / np.cos(alpha_wa)) - 1)
    # Tip circle diameters
    das  = MA * z_s + 2 * MA * (1 + ya - xp1)
    dap1 = MA * z_p1 + 2 * MA * (1 + np.minimum(ya - xs, xp1))
    dar1 = MA * target_z_r1 - 2 * MA * (1 - XR1)
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
    Ea_val = 1 - mu* np.pi * (1 / z_s + 1 / z_p1) * ea
    Eb_val = 1 - mu* np.pi * (1 / z_p1 - 1 / target_z_r1) * eb
    Ec_val = 1 - mu* np.pi * (1 / z_p2 - 1 / z_r2) * ec
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
takes:  in_vals (parameters to sweep) [5,1] list of arrays - [z_sh, z_r2, xs, xr2, Cl] OR [4,1] if add_cl is True.
        add_gear - boolean whether to also add in the values from Matsuki 2019 paper
        add_cl - boolean whether to also add in set Cl values (False means they will be given)
gives: score [scalar] - number of valid combinations
NOTE - for scores to be transferable, give the same size vectors across different in_vals

'''

def score_vals(in_vals, offsets = None, add_gear = True, return_verbose = False, mu = MU, ):

    # print(in_vals)

    # 2019 Matsuki 'Table 3' values
    paper_vals = [6, 81, .476, 1.0, 0.1e-3, 0, 0] # NOTE - 6 = 12 (z_s) / 2. 1.0 is as close as we can get to x_r2 = 1.2

    if add_gear:
        print(f'We\'re adding a gear')
        for i in range(len(in_vals)):
            in_vals[i] = np.hstack([in_vals[i], paper_vals[i]])

    # print(in_vals)

    # validated_design_vals = z_sh, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2, r_a, I1, I2, gr_s, ratios
    validated_design_vals, I1, I2, gr_s, ratios = validate_parameters(in_vals, offsets = offsets)

    eta_fwd, eta_bwd = efficiency_calc(validated_design_vals, mu=mu)

    # composite_score = ratios * eta_fwd * eta_bwd # possibly useful in future ?
    bd_indices = np.where(eta_bwd > 0.3)

    score = len(validated_design_vals[0][bd_indices]) # how many valid z_s (valid sets total)?
    gr_score = np.mean(ratios[bd_indices])*len(validated_design_vals[0][bd_indices]) # average gear ratio (among working) * number of valid z_s (valid sets)
    # print(f'Score compare: best - {best_score}, current - {score}')len(validated_design_vals[0][bd_indices])
    if return_verbose:
        return gr_score, validated_design_vals, eta_bwd, ratios
    return gr_score

'''
Function 'param_to_list'
takes:  params - dictionary from Ax.dev detailing parameters
        add_cl - boolean whether to also add in set Cl values (False means they will be given)
        w_offset - boolean whether the Ax values will include 2 offsets (p1_offset, p2_offset)
gives: parm_list [5,n] list for use in score_vals  - [z_sh, z_r2, xs, xr2, Cl]

'''
def param_to_list(param, add_cl = False, add_offset = False):

    num_param = 5 # default number of parameters
    if add_cl:
        num_param -= 1 # add in set Cl values

    each_val_len = len(param) // num_param # n = 3 parameters (usually)

    param_list = []
    for i in range(num_param):
        param_list.append(np.array(list(param.values())[i*each_val_len:(i+1)*each_val_len]))
    
    if add_cl:
        # add in set Clearance values
        if each_val_len == 3: # should be deprecated.
            set_Cl = np.array([0, 0.2e-3, 0.3e-3])
        elif each_val_len == 4:
            set_Cl = np.array([0, 0.1e-3, 0.2e-3, 0.3e-3])
        param_list.append(set_Cl)
    if add_offset:
        offsets = np.array([[-1, 0, 1, 2],[-1, 0, 1, 2]])
        return param_list, offsets
    else:
        return param_list


def list_to_param(param_list, vals_per = 4):
    """
    Function 'list_to_param'
    takes: param_list [5,1] list for use in score_vals - [z_sh, z_r2, xs, xr2, Cl, p1_offset, p2_offset]
    gives: params - dictionary from Ax.dev detailing parameters
    """

    if len(param_list) == 4:
        naming = ['z_sh', 'z_r2', 'xs', 'xr2']
    elif len(param_list) == 5:
        naming = ['z_sh', 'z_r2', 'xs', 'xr2', 'Cl']
    elif len(param_list) == 6:
        naming = ['z_sh', 'z_r2', 'xs', 'xr2', 'p1_offset', 'p2_offset']
    elif len(param_list) == 7:
        naming = ['z_sh', 'z_r2', 'xs', 'xr2', 'Cl', 'p1_offset', 'p2_offset']


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
        elif naming[ind][0] == 'x' or naming[ind][0] == 'C':
            if len(param_list[ind]) == 1:
                param[naming[ind]] = float(param_list[ind][0])
                continue
            else:
                param[naming[ind]] = float(param_list[ind][0])
                for ind2 in range(1, vals_per):
                    param[naming[ind] + '_' + str(ind2)] = float(param_list[ind][ind2])

    return param

def plot_efficiencies(data):
    """
    Creates a detailed 3D scatter plot of overall gear ratio, forward efficiency,
    and backward efficiency.
    Hovering over a dot displays detailed candidate information.
    """
    import warnings
    warnings.filterwarnings("ignore", message="3d coordinates not supported", category=UserWarning)
    
    filtered = [d for d in data if d[7] > 0 and d[8] > 0]
    if not filtered:
        print("No valid data to plot.")
        return

    gear_ratios, fwd_effs, bwd_effs, labels = [], [], [], []
    for d in filtered:
        # candidate: (iteration, z_s, z_p1, z_r1, z_p2, z_r2, gear_ratio, eff_fwd, eff_bwd, composite)
        itr, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, eff_fwd, eff_bwd, _ = d
        label = (f"Iteration: {itr}\n"
                 f"Sun (z_s): {z_s}, Planet 1 (z_p1): {z_p1}, Ring 1 (z_r1): {z_r1}\n"
                 f"Planet 2 (z_p2): {z_p2}, Ring 2 (z_r2): {z_r2}\n"
                 f"Gear Ratio: 1:{overall_ratio:.2f}\n"
                 f"Forward Eff: {eff_fwd*100:.2f}%\n"
                 f"Backward Eff: {eff_bwd*100:.2f}%")
        gear_ratios.append(overall_ratio)
        fwd_effs.append(eff_fwd*100)
        bwd_effs.append(eff_bwd*100)
        labels.append(label)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(gear_ratios, fwd_effs, bwd_effs, c=fwd_effs, cmap='viridis',
                    edgecolor='k', alpha=0.8, s=60)
    
    ax.set_xlabel("Gear Ratio (1:ratio)")
    ax.set_ylabel("Forward Efficiency (%)")
    ax.set_zlabel("Backward Efficiency (%)")
    ax.set_title("Detailed Efficiency Plot")
    cursor = mplcursors.cursor(sc, hover=True)
    @cursor.connect("add")
    def on_add(sel):
        index = sel.index
        sel.annotation.set_text(labels[index])
        sel.annotation.get_bbox_patch().set(fc="whitesmoke", alpha=0.9)
    fig.colorbar(sc, label="Forward Efficiency (%)")
    plt.show()