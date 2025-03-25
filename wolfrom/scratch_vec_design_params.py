"""
This script is used to compute design parameter options for the Wolfrom Gearbox
Usage: 
    - Modify the config.json file to set the design parameters
    - Modify target gear ratio, ratio tolerance, and display flags as needed
    - Run the script to compute the gearbox design options
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
from gearbox import Gearbox

config_path = os.path.join(os.path.dirname(__file__), "config.json")
with open(config_path, "r") as f:
    config = json.load(f)

MODULE = config["MODULE"]
MU = config["MU"]
MC = config.get("MC", MODULE)
ALPHA = np.radians(config.get("ALPHA", 20))
XR1 = config.get("XR1", 0.0)
N_PLANETS = config["N_PLANETS"]
SUN_LIMITS = tuple(config["SUN_LIMITS"])
P1_LIMITS  = tuple(config["P1_LIMITS"])
P2_LIMITS  = tuple(config["P2_LIMITS"])
R2_LIMITS  = tuple(config["R2_LIMITS"])
STEPS = tuple(config["STEPS"])
MIN_RATIO_THRESHOLD = config["MIN_RATIO_THRESHOLD"]
MIN_FORWARD_THRESHOLD = config["MIN_FORWARD_THRESHOLD"]
MIN_BACKWARD_THRESHOLD = config["MIN_BACKWARD_THRESHOLD"]

SLICE_GEAR_RATIO = True 
TARGET_GEAR_RATIO = 100
RATIO_TOLERANCE = 10
DISPLAY_BEST_GEARBOX = False
DISPLAY_ITERATIONS_PLOT = True
RUN_SAMPLE_TRIAL = False
MAX_CANDIDATES = 400

def generate_range_options(range_vals):
    low, high, step = range_vals
    opts = []
    current = low
    while current <= high + 1e-9:
        opts.append(round(current, 5))
        current += step
    return opts

def update_tracker_list(tracker, value, candidate):
    tracker.append((value, candidate))
    tracker.sort(key=lambda x: x[0], reverse=True)
    return tracker[:MAX_CANDIDATES]

class GearAnalysis:  
    def __init__(self, sun_options, p1_options, p2_options, r2_options,
                 xs_options, xp1_options, xp2_options, xr2_options,
                 min_ratio_threshold=MIN_RATIO_THRESHOLD, 
                 min_forward_threshold=MIN_FORWARD_THRESHOLD, 
                 min_backward_threshold=MIN_BACKWARD_THRESHOLD):
        self.sun_options = sun_options
        self.p1_options = p1_options
        self.p2_options = p2_options
        self.r2_options = r2_options
        self.xs_options = xs_options
        self.xp1_options = xp1_options
        self.xp2_options = xp2_options
        self.xr2_options = xr2_options
        self.min_ratio_threshold = min_ratio_threshold
        self.min_forward_threshold = min_forward_threshold
        self.min_backward_threshold = min_backward_threshold
        self.iteration = 0
        self.best_forward_overall = []
        self.best_backward_overall = []
        self.best_ratio_overall = []
        self.best_composite_overall = []
        self.best_forward_meet = []
        self.best_backward_meet = []
        self.best_ratio_meet = []
        self.best_composite_meet = []
        self.iterations_data = []  
    
    def old_compute(self):
        total_iterations = (len(self.sun_options) * len(self.p1_options) * len(self.p2_options) *
                            len(self.r2_options) * len(self.xs_options) * len(self.xp1_options) *
                            len(self.xp2_options) * len(self.xr2_options))
        from tqdm import tqdm
        with tqdm(total=total_iterations, desc="Computing gearbox", unit="iter") as pbar:
            for z_s in self.sun_options:
                for z_p1 in self.p1_options:
                    z_r1 = z_s + 2 * z_p1
                    for z_p2 in self.p2_options:
                        for z_r2 in self.r2_options:
                            for xs in self.xs_options:
                                for xp1 in self.xp1_options:
                                    for xp2 in self.xp2_options:
                                        for xr2 in self.xr2_options:
                                            self.iteration += 1
                                            gear = Gearbox(z_s, z_p1, z_r1, z_p2, z_r2,
                                                           xs=xs, xp1=xp1, xp2=xp2,
                                                           xr1=config["XR1"], xr2=xr2)
                                            gr = gear.gear_ratio()
                                            overall_ratio = 1.0 / gr
                                            if gr <= 0 or np.abs(TARGET_GEAR_RATIO - overall_ratio) > RATIO_TOLERANCE: 
                                                #let's be more restrictive with gear ratio to increase code speed. GMC 3/25
                                                pbar.update(1)
                                                continue
                                            try:
                                                eff_fwd = gear.forward_efficiency()
                                                eff_bwd = gear.backward_efficiency()
                                            except NotImplementedError:
                                                pbar.update(1)
                                                continue
                                            if eff_fwd < 0 or eff_bwd < 0:
                                                pbar.update(1)
                                                continue
                                            if eff_fwd > 2.0 or eff_bwd > 2.0:
                                                pbar.update(1)
                                                continue
                                            composite_score = overall_ratio * eff_fwd * eff_bwd
                                            candidate = (self.iteration, z_s, z_p1, z_r1, z_p2, z_r2,
                                                         overall_ratio, eff_fwd, eff_bwd, composite_score,
                                                         xs, xp1, xp2, xr2)
                                            self.iterations_data.append(candidate)
                                            self.best_forward_overall = update_tracker_list(self.best_forward_overall, eff_fwd, candidate)
                                            self.best_backward_overall = update_tracker_list(self.best_backward_overall, eff_bwd, candidate)
                                            self.best_ratio_overall = update_tracker_list(self.best_ratio_overall, overall_ratio, candidate)
                                            self.best_composite_overall = update_tracker_list(self.best_composite_overall, composite_score, candidate)
                                            if overall_ratio >= self.min_ratio_threshold and eff_fwd >= self.min_forward_threshold and eff_bwd >= self.min_backward_threshold:
                                                self.best_forward_meet = update_tracker_list(self.best_forward_meet, eff_fwd, candidate)
                                                self.best_backward_meet = update_tracker_list(self.best_backward_meet, eff_bwd, candidate)
                                                self.best_ratio_meet = update_tracker_list(self.best_ratio_meet, overall_ratio, candidate)
                                                self.best_composite_meet = update_tracker_list(self.best_composite_meet, composite_score, candidate)
                                            pbar.update(1)

    def compute(self):
        sun_options = np.array(self.sun_options)
        p1_options = np.array(self.p1_options)
        p2_options = np.array(self.p2_options)
        r2_options = np.array(self.r2_options)
        xs_options = np.array(self.xs_options)
        xp1_options = np.array(self.xp1_options)
        xp2_options = np.array(self.xp2_options)
        xr2_options = np.array(self.xr2_options)

        z_s, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2 = np.meshgrid(
            sun_options, p1_options, p2_options, r2_options, xs_options, xp1_options, xp2_options, xr2_options, indexing='ij'
        )

        total_combinations = z_s.size
        self.iteration = 0

        # flatten everything
        z_s, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2 = z_s.flatten(), z_p1.flatten(), z_p2.flatten(), z_r2.flatten(), xs.flatten(), xp1.flatten(), xp2.flatten(), xr2.flatten()

        z_r1 = z_s + 2 * z_p1

        I1 = z_r1 / z_s
        I2 = (z_r1 * z_p2) / (z_p1 * z_r2)
        gr_s = (1 - I2) / (1 + I1)
        ratios = 1 / gr_s
        # I1, I2, gr_s, ratios = I1.flatten(), I2.flatten(), gr_s.flatten(), ratios.flatten()


        # plot gr_s and ratios (subsampled) in 2 subplots
        subsample = 1000
        plot_grs = gr_s[::subsample]
        plot_ratios = ratios[::subsample]
        fig, axs = plt.subplots(figsize=(12, 6))
        len_plot = len(plot_grs)
        axs.scatter(plot_grs, plot_ratios)
        plt.show()

        # only keep valid combinations (ratios w.in tolerance)
        valid_combinations = np.logical_and(gr_s > 0, np.abs(TARGET_GEAR_RATIO - ratios) <= RATIO_TOLERANCE)
        
        z_r1 = z_r1[valid_combinations]
        z_s, z_p1, z_p2, z_r2, xs, xp1, xp2, xr2 = z_s[valid_combinations], z_p1[valid_combinations], z_p2[valid_combinations], z_r2[valid_combinations], xs[valid_combinations], xp1[valid_combinations], xp2[valid_combinations], xr2[valid_combinations]
        I1, I2, gr_s, ratios = I1[valid_combinations], I2[valid_combinations], gr_s[valid_combinations], ratios[valid_combinations]

        assert z_s.shape == z_p1.shape == z_r1.shape == z_p2.shape == z_r2.shape == xs.shape == xp1.shape == xp2.shape == xr2.shape == ratios.shape

        print(f'Of {total_combinations} total inputs, there are {len(z_s)} valid combinations')

        ra = (z_s * MODULE + z_p1 * MODULE) / 2.0

        # Basic circle diameters
        db_a1 = MODULE * z_s * np.cos(ALPHA)
        db_a2 = MODULE * z_p1 * np.cos(ALPHA)
        db_b1 = MODULE * z_p1 * np.cos(ALPHA)
        db_b2 = MODULE * z_r1 * np.cos(ALPHA)
        db_c1 = MC * z_p2 * np.cos(ALPHA)
        db_c2 = MC * z_r2 * np.cos(ALPHA)

        # Working pressure angles (modified formula)
        alpha_wa = np.arccos(MODULE * (z_s + z_p1) * np.cos(ALPHA) / (2 * ra))
        alpha_wb = np.arccos(MODULE * (-z_p1 + z_r1) * np.cos(ALPHA) / (2 * ra))
        val = MC * (-z_p2 + z_r2) * np.cos(ALPHA) / (2 * ra)
        alpha_wc = np.arccos(np.clip(val, -1, 1))
        # Center distance modification coefficient
        ya = ((z_s + z_p1) / 2.0) * ((np.cos(ALPHA) / np.cos(alpha_wa)) - 1)
        # Tip circle diameters
        das  = MODULE * z_s + 2 * MODULE * (1 + ya - xp1)
        dap1 = MODULE * z_p1 + 2 * MODULE * (1 + np.minimum(ya - xs, xp1))
        dar1 = MODULE * z_r1 - 2 * MODULE * (1 - XR1)
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
        eb1 = -(z_r1 / (2 * np.pi)) * (np.tan(alpha_ab2) - np.tan(alpha_wb))
        eb2 = (z_p1 / (2 * np.pi)) * (np.tan(alpha_ab1) - np.tan(alpha_wb))
        ec1 = -(z_r2 / (2 * np.pi)) * (np.tan(alpha_ac2) - np.tan(alpha_wc))
        ec2 = (z_p2 / (2 * np.pi)) * (np.tan(alpha_ac1) - np.tan(alpha_wc))
        ea = ea1**2 + ea2**2 - ea1 - ea2 + 1
        eb = eb1**2 + eb2**2 - eb1 - eb2 + 1
        ec = ec1**2 + ec2**2 - ec1 - ec2 + 1
        # Basic driving efficiencies
        Ea_val = 1 - MU* np.pi * (1 / z_s + 1 / z_p1) * ea
        Eb_val = 1 - MU* np.pi * (1 / z_p1 - 1 / z_r1) * eb
        Ec_val = 1 - MU* np.pi * (1 / z_p2 - 1 / z_r2) * ec
        # Forward driving efficiency 
        eta_fwd = (1 + Ea_val * Eb_val * I1) * (1 - I2) / ((1 + I1) * (1 - Eb_val * Ec_val * I2))
        # Backward driving efficiency
        eta_bwd = (1 + I1) * Ea_val * (Eb_val * Ec_val - I2) / (Ec_val * (Ea_val * Eb_val + I1) * (1 - I2))

        # Average and std for eta
        avg_fwd = np.mean(eta_fwd)
        std_fwd = np.std(eta_fwd)
        avg_bwd = np.mean(eta_bwd)
        std_bwd = np.std(eta_bwd)
        print(f'Efficiency shapes: {eta_fwd.shape}, {eta_bwd.shape}')
        print(f"Average Forward Efficiency: {avg_fwd:.4f} +/- {std_fwd:.4f}")
        print(f"Average Backward Efficiency: {avg_bwd:.4f} +/- {std_bwd:.4f}")

        composite_score = ratios * eta_fwd * eta_bwd

        # create a list of the n (Max_Candidates) best scores
        # sort all vectors by composite score
        sorted_indices = np.argsort(composite_score)[::-1]
        best_indices = sorted_indices[:MAX_CANDIDATES]
        
        # make 'best' versions of all the arrays
        best_z_s = z_s[best_indices]
        best_z_p1 = z_p1[best_indices]
        best_z_r1 = z_r1[best_indices]
        best_z_p2 = z_p2[best_indices]
        best_z_r2 = z_r2[best_indices]
        best_xs = xs[best_indices]
        best_xp1 = xp1[best_indices]
        best_xp2 = xp2[best_indices]
        best_xr2 = xr2[best_indices]
        best_ratios = ratios[best_indices]
        best_eta_fwd = eta_fwd[best_indices]
        best_eta_bwd = eta_bwd[best_indices]
        best_composite_score = composite_score[best_indices]
        

        # plot: Gear ratio, forward efficiency, backward efficiency (color forward efficiency)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        sc = ax.scatter(best_ratios, best_eta_fwd, c=best_eta_bwd, cmap='viridis', edgecolor='k', alpha=1, s=50)
        ax.set_xlabel("Gear Ratio (1:ratio)")
        ax.set_ylabel("Forward Efficiency (%)")
        ax.set_title("Gear Ratio vs. Forward Efficiency")
        fig.colorbar(sc, label="Forward Efficiency (%)")
        plt.show()

  
def compute_gearbox():
    """
    Compute the 3K Planetary Gearbox by iterating over design options,
    then report metrics and plot results.
    """
    start_time = time.time()
    options = generate_all_options(SUN_LIMITS, P1_LIMITS, P2_LIMITS, R2_LIMITS, STEPS)
    sun_options = options["sun"]
    p1_options  = options["p1"]
    p2_options  = options["p2"]
    r2_options  = options["r2"]
    xs_options = generate_range_options(config["XS_RANGE"])
    xp1_options = generate_range_options(config["XP1_RANGE"])
    xp2_options = generate_range_options(config["XP2_RANGE"])
    xr2_options = generate_range_options(config["XR2_RANGE"])
    
    total_iter = (len(sun_options) * len(p1_options) * len(p2_options) * len(r2_options) *
                  len(xs_options) * len(xp1_options) * len(xp2_options) * len(xr2_options))
    if RUN_SAMPLE_TRIAL:
        sample_count = 100
        avg_iter_time = measure_sample_avg_time(sample_count, sun_options, p1_options, p2_options, r2_options,
                                                xs_options, xp1_options, xp2_options, xr2_options)
        print(f"Sample trial enabled: Average time per iteration (sample of {sample_count}): {avg_iter_time:.6f} seconds")
    else:
        avg_iter_time = 265.43 / 2722500 # Average time per iteration from trial run
        print(f"Using default average time per iteration: {avg_iter_time:.6f} seconds")
    predicted_time = total_iter * avg_iter_time
    print(f"Total iterations to run: {total_iter}")
    print(f"Predicted total computation time: {predicted_time:.2f} seconds.")
    input("Press Enter to run script...")

    gear_analysis = GearAnalysis(sun_options, p1_options, p2_options, r2_options,
                                 xs_options, xp1_options, xp2_options, xr2_options)
    gear_analysis.compute()

def generate_options(lower: int, upper: int, step: int = 1):
    return list(range(lower, upper + 1, step))

def generate_all_options(sun_limits: tuple, p1_limits: tuple, p2_limits: tuple, r2_limits: tuple, 
                         steps: tuple = (1,1,1,1)):
    step_sun, step_p1, step_p2, step_r2 = steps
    return {
        "sun": generate_options(sun_limits[0], sun_limits[1], step_sun),
        "p1": generate_options(p1_limits[0], p1_limits[1], step_p1),
        "p2": generate_options(p2_limits[0], p2_limits[1], step_p2),
        "r2": generate_options(r2_limits[0], r2_limits[1], step_r2)
    }

if __name__ == "__main__":
    print("Starting computation based on defined parameters...\n")
    compute_gearbox()
