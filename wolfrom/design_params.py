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
MAX_CANDIDATES = 50

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

    def compute(self):
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
                                            if gr <= 0:
                                                pbar.update(1)
                                                continue
                                            overall_ratio = 1.0 / gr
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

    def print_metric(self, metric_name, tracker, threshold):
        if not tracker:
            print(f"\nNo combination meets {metric_name} threshold of {threshold}.")
        else:
            best_value, best_candidate = tracker[0]
            print(f"{metric_name} (req satisfied, top candidate of {len(tracker)} kept): Iteration: {best_candidate[0]} | Sun:{best_candidate[1]} (xs:{best_candidate[10]}), "
                  f"P1:{best_candidate[2]} (xp1:{best_candidate[11]}), R1:{best_candidate[3]}, "
                  f"P2:{best_candidate[4]} (xp2:{best_candidate[12]}), R2:{best_candidate[5]} (xr2:{best_candidate[13]}) => "
                  f"Gear Ratio: 1:{best_candidate[6]:.2f}, Fwd: {best_candidate[7]*100:.2f}%, "
                  f"Bwd: {best_candidate[8]*100:.2f}%, Composite: {best_candidate[9]:.2e}")

    def report(self):
        print("\nBest Forward Efficiency:")
        self.print_metric("Forward Efficiency", self.best_forward_meet, self.min_forward_threshold)
        print("\nBest Backward Efficiency:")
        self.print_metric("Backward Efficiency", self.best_backward_meet, self.min_backward_threshold)
        print("\nBest Gear Ratio:")
        self.print_metric("Gear Ratio", self.best_ratio_meet, self.min_ratio_threshold)
        print("\nBest Overall Combination (Composite Score):")
        self.print_metric("Composite Score", self.best_composite_meet, "N/A")

def plot_efficiencies(data, total_iterations, slice_flag=False, target_ratio=TARGET_GEAR_RATIO, tolerance=RATIO_TOLERANCE):
    """
    Creates a 3D scatter plot showing only candidates that meet the target gear ratio requirement
    within the specified tolerance. Points are colored by their forward efficiency.
    """
    import warnings
    warnings.filterwarnings("ignore", message="3d coordinates not supported yet", category=UserWarning)
    
    lower_bound = target_ratio * (1 - tolerance / 100.0)
    upper_bound = target_ratio * (1 + tolerance / 100.0)
    
    filtered = [d for d in data if d[7] > 0 and d[8] > 0 
                and d[6] >= lower_bound and d[6] <= upper_bound]
    
    if not filtered:
        print("No candidates meet the target gear ratio requirement.")
        return

    gear_ratios = []
    fwd_eff = []
    bwd_eff = []
    labels = []
    
    for d in filtered:
        itr, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, eff_fwd, eff_bwd, _, xs, xp1, xp2, xr2 = d
        gear_ratios.append(overall_ratio)
        fwd_eff.append(eff_fwd * 100)
        bwd_eff.append(eff_bwd * 100)
        
        label = (f"Iteration: {itr}\n"
                f"Sun: {z_s} (xs:{xs})\n"
                f"P1: {z_p1} (xp1:{xp1}), R1: {z_r1}\n"
                f"P2: {z_p2} (xp2:{xp2}), R2: {z_r2} (xr2:{xr2})\n"
                f"Gear Ratio: 1:{overall_ratio:.2f}\n"
                f"Fwd: {eff_fwd*100:.2f}%, Bwd: {eff_bwd*100:.2f}%")
        labels.append(label)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    sc = ax.scatter(gear_ratios, fwd_eff, bwd_eff, c=fwd_eff, cmap='viridis', edgecolor='k', alpha=1, s=50)
    
    ax.set_xlabel("Gear Ratio (1:ratio)")
    ax.set_ylabel("Forward Efficiency (%)")
    ax.set_zlabel("Backward Efficiency (%)")
    ax.set_title(f"Target Ratio: {target_ratio}±{tolerance}%")
    
    global_text = (
        f"Total Iterations: {total_iterations}\n"
        f"Target Gear Ratio: {target_ratio}±{tolerance}%\n"
        f"Friction Coefficient: {MU}\n"
        f"Sun: [{SUN_LIMITS[0]}, {SUN_LIMITS[1]}]\n"
        f"Planet1: [{P1_LIMITS[0]}, {P1_LIMITS[1]}]\n"
        f"Planet2: [{P2_LIMITS[0]}, {P2_LIMITS[1]}]\n"
        f"Ring2: [{R2_LIMITS[0]}, {R2_LIMITS[1]}]\n"
        f"Step Sizes: {STEPS}\n"
    )
    anchored_text = AnchoredText(
        global_text, loc='upper left',
        bbox_to_anchor=(0, 1),
        bbox_transform=plt.gcf().transFigure,
        prop=dict(size=10),
        frameon=True
    )
    ax.add_artist(anchored_text)
    
    cursor = mplcursors.cursor(sc, hover=True)
    def on_add(sel):
        sel.annotation.set_visible(True)
        sel.annotation.get_bbox_patch().set_facecolor("lightgreen")
        sel.annotation.set_text(labels[sel.index])
    cursor.connect("add", on_add)
    
    fig.colorbar(sc, label="Forward Efficiency (%)")
    plt.show()

def measure_sample_avg_time(iter_count, sun_options, p1_options, p2_options, r2_options,
                            xs_options, xp1_options, xp2_options, xr2_options):
    import time
    sample_sun = sun_options[0]
    sample_p1 = p1_options[0]
    sample_p2 = p2_options[0]
    sample_r2 = r2_options[0]
    sample_xs = xs_options[0]
    sample_xp1 = xp1_options[0]
    sample_xp2 = xp2_options[0]
    sample_xr2 = xr2_options[0]
    sample_r1 = sample_sun + 2 * sample_p1
    start_sample = time.time()
    for i in range(iter_count):
        gear = Gearbox(sample_sun, sample_p1, sample_r1, sample_p2, sample_r2,
                       xs=sample_xs, xp1=sample_xp1, xp2=sample_xp2,
                       xr1=config["XR1"], xr2=sample_xr2)
        _ = gear.forward_efficiency()
        _ = gear.backward_efficiency()
    end_sample = time.time()
    return (end_sample - start_sample) / iter_count

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
    print("\nParameters:")
    print(f"Module: {MODULE}, Friction Coefficient: {MU}, Number of Planets: {N_PLANETS}")
    gear_analysis.report()
    
    best_candidate = gear_analysis.best_composite_meet[0][1] if gear_analysis.best_composite_meet else None
    if best_candidate is None:
        print("No candidate satisfies the thresholds. Skipping plotting.")
    else:
        pass

    if DISPLAY_ITERATIONS_PLOT:
        end_time = time.time() 
        total_time = end_time - start_time
        print(f"Total computation time: {total_time:.2f} seconds")
        
        if gear_analysis.best_forward_meet:
            print("\nPlotting efficiency results for best forward candidates...")
            plot_efficiencies(
                [pair[1] for pair in gear_analysis.best_forward_meet],
                gear_analysis.iteration,
                slice_flag=SLICE_GEAR_RATIO,
                target_ratio=TARGET_GEAR_RATIO,
                tolerance=RATIO_TOLERANCE
            )
        else:
            print("No best forward candidates to plot.")
            
        if gear_analysis.best_backward_meet:
            print("\nPlotting efficiency results for best backward candidates...")
            plot_efficiencies(
                [pair[1] for pair in gear_analysis.best_backward_meet],
                gear_analysis.iteration,
                slice_flag=SLICE_GEAR_RATIO,
                target_ratio=TARGET_GEAR_RATIO,
                tolerance=RATIO_TOLERANCE
            )
        else:
            print("No best backward candidates to plot.")

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
