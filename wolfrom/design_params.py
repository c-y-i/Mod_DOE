"""
This script is used to compute design parameter options for the Wolfrom Gearbox
Usage: 
    - Modify the config.json file to set the design parameters
    - Modify target gear ratio, ratio tolerance, and display flags as needed
    - Run the script to compute the gearbox design options
"""
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import json
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import csv
from tqdm import tqdm
from utils import validate_parameters, efficiency_calc, plot_efficiencies, TARG_zr1

# load config
config_path = os.path.join(os.path.dirname(__file__), "config.json")
with open(config_path, "r") as f:
    config = json.load(f)

MODULE = config["MODULE"] # M_A (Stage 1)
MU = config["MU"]
MC = config.get("MC", MODULE) # Stage 2
ALPHA = np.radians(config.get("ALPHA", 20))
XR1 = 0
N_PLANETS = config["N_PLANETS"]
SUN_LIMITS = tuple(config["SUN_LIMITS"])
P1_LIMITS  = tuple(config["P1_LIMITS"])
P2_LIMITS  = tuple(config["P2_LIMITS"])
R2_LIMITS  = tuple(config["R2_LIMITS"])
STEPS = tuple(config["STEPS"])
RATIO_THRESHOLD = config["TARGET_GEAR_RATIO"]
RATIO_TOLERANCE = config["RATIO_TOLERANCE"]
MIN_FORWARD_THRESHOLD = config["MIN_FORWARD_THRESHOLD"]
MIN_BACKWARD_THRESHOLD = config["MIN_BACKWARD_THRESHOLD"]

RUN_SAMPLE_TRIAL = False
MAX_CANDIDATES = 50000

# Number of top combinations to display in terminal
TOP_COMBINATIONS = 100

def generate_options(lower: int, upper: int, step: int = 1):
    return list(range(lower, upper + 1, step))

def generate_range_options(range_vals):
    low, high, step = range_vals
    opts = []
    current = low
    while current <= high + 1e-9:
        opts.append(round(current, 5))
        current += step
    return opts

def generate_all_options(sun_limits: tuple, p1_limits: tuple, p2_limits: tuple, r2_limits: tuple, 
                         steps: tuple = (1,1,1,1)):
    step_sun, step_p1, step_p2, step_r2 = steps
    return {
        "sun": generate_options(sun_limits[0], sun_limits[1], step_sun),
        "p1": generate_options(p1_limits[0], p1_limits[1], step_p1),
        "p2": generate_options(p2_limits[0], p2_limits[1], step_p2),
        "r2": generate_options(r2_limits[0], r2_limits[1], step_r2)
    }

options = generate_all_options(SUN_LIMITS, P1_LIMITS, P2_LIMITS, R2_LIMITS, STEPS)
sun_options = options["sun"]
p1_options  = options["p1"]
p2_options  = options["p2"]
r2_options  = options["r2"]
xs_options = generate_range_options(config["XS_RANGE"])
xp1_options = generate_range_options(config["XP1_RANGE"])
xp2_options = generate_range_options(config["XP2_RANGE"])
xr2_options = generate_range_options(config["XR2_RANGE"])

def view_vector(input_vector, subsample, title, y_lims=None):
    fig, ax = plt.subplots()
    to_plot = input_vector[::subsample]
    xs = np.arange(len(to_plot))
    ax.scatter(xs, to_plot)
    ax.set_title(title)
    if y_lims:
        ax.set_ylim(y_lims)
    plt.show()


z_sh_options = np.array(sun_options) / 2  # convert sun_options to z_sh
r2_options = np.array(r2_options)         # use r2_options as-is
xs_options = np.array(xs_options)
xr2_options = np.array(xr2_options)
# choose clearance values (example: three clearance options)
clearance_options = np.array([0, 0.1e-3, 0.2e-3, 0.3e-3])

# Get design parameters and calculated values from utils with progress bar
with tqdm(total=3, desc="Design evaluation") as pbar:
    design_vals, I1, I2, gr_s, ratios = validate_parameters([
        z_sh_options, 
        r2_options, 
        xs_options, 
        xr2_options, 
        clearance_options
    ])
    pbar.update(1)
    eta_fwd, eta_bwd = efficiency_calc(design_vals)
    composite_score = ratios * eta_fwd * eta_bwd
    pbar.update(1)
    sorted_indices = np.argsort(composite_score)[::-1]
    best_indices = sorted_indices[:MAX_CANDIDATES]
    pbar.update(1)

# Unpack design values: [z_sh, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2, r_a]
best_z_sh = design_vals[0][best_indices]
best_z_p1 = design_vals[1][best_indices]
best_z_p2 = design_vals[2][best_indices]
best_z_r2 = design_vals[3][best_indices]
best_xs   = design_vals[4][best_indices]
best_xp1  = design_vals[5][best_indices]
best_xp2  = design_vals[6][best_indices]
best_xr2  = design_vals[7][best_indices]
best_r_a  = design_vals[8][best_indices]
best_ratios = ratios[best_indices]
best_eta_fwd = eta_fwd[best_indices]
best_eta_bwd = eta_bwd[best_indices]

# Filter results by target ratio ± tolerance and minimum forward efficiency
valid_mask = (best_ratios >= (RATIO_THRESHOLD - RATIO_TOLERANCE)) & \
             (best_ratios <= (RATIO_THRESHOLD + RATIO_TOLERANCE)) & \
             (best_eta_fwd >= MIN_FORWARD_THRESHOLD)
best_z_sh = best_z_sh[valid_mask]
best_z_p1 = best_z_p1[valid_mask]
best_z_p2 = best_z_p2[valid_mask]
best_z_r2 = best_z_r2[valid_mask]
best_xs   = best_xs[valid_mask]
best_xp1  = best_xp1[valid_mask]
best_xp2  = best_xp2[valid_mask]
best_xr2  = best_xr2[valid_mask]
best_r_a  = best_r_a[valid_mask]
best_ratios = best_ratios[valid_mask]
best_eta_fwd = best_eta_fwd[valid_mask]
best_eta_bwd = best_eta_bwd[valid_mask]


# Compute z_s as 2x z_sh
z_s = 2 * best_z_sh

print(f'Of {len(ratios)} total combinations, there are {len(best_z_sh)} valid ones')
view_vector(eta_fwd, 1, "Forward Efficiency")
view_vector(eta_bwd, 1, "Backward Efficiency")
output_dir = os.path.join(os.path.dirname(__file__), "results")
os.makedirs(output_dir, exist_ok=True)
csv_filename = os.path.join(output_dir, "valid_combinations.csv")
with open(csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["z_s", "z_p1", "z_p2", "z_r2", "xs", "xp1", "xp2", "xr2", "Ratio", "eta_fwd", "eta_bwd"])
    for i in range(min(len(z_s), len(best_ratios))):
        writer.writerow([
            z_s[i],
            best_z_p1[i],
            best_z_p2[i],
            best_z_r2[i],
            best_xs[i],
            best_xp1[i],
            best_xp2[i],
            best_xr2[i],
            best_ratios[i],
            best_eta_fwd[i],
            best_eta_bwd[i]
        ])

print(f"\nResults saved to CSV: {csv_filename}")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(best_ratios, best_eta_fwd, best_eta_bwd, c=best_eta_fwd, cmap='viridis',
                edgecolor='k', alpha=1, s=50)
ax.set_xlabel("Gear Ratio (1:ratio)")
ax.set_ylabel("Forward Efficiency")
ax.set_zlabel("Backward Efficiency")
ax.set_title("Gear Ratio vs. Forward Efficiency")
fig.colorbar(sc, label="Forward Efficiency")
plt.show()

# using the plotting function from utils to visualize the results
candidates = []
for i in range(len(best_ratios)):
    z_s_val = 2 * best_z_sh[i]
    candidate = (i, z_s_val, best_z_p1[i], TARG_zr1, best_z_p2[i], best_z_r2[i],
                 best_ratios[i], best_eta_fwd[i], best_eta_bwd[i], None)
    candidates.append(candidate)

plot_efficiencies(candidates)
