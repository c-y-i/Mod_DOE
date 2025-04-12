"""
    Analysis of a 3K Planetary Gearbox
    features added in this version:
    - Gearbox class now accepts additional parameters for profile shift coefficients.
    - GearAnalysis class now includes option to slice the 3D efficiency plot based on a target gear ratio.
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

# Load configuration from config.json
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

# slicing parameters:
SLICE_GEAR_RATIO = True 
TARGET_GEAR_RATIO = 100.0 
RATIO_TOLERANCE = 10.0 

DISPLAY_BEST_GEARBOX = False
DISPLAY_ITERATIONS_PLOT = True

class GearAnalysis:  
    def __init__(self, sun_options, p1_options, p2_options, r2_options,
                 min_ratio_threshold=MIN_RATIO_THRESHOLD, 
                 min_forward_threshold=MIN_FORWARD_THRESHOLD, 
                 min_backward_threshold=MIN_BACKWARD_THRESHOLD):
        self.sun_options = sun_options
        self.p1_options = p1_options
        self.p2_options = p2_options
        self.r2_options = r2_options
        self.min_ratio_threshold = min_ratio_threshold
        self.min_forward_threshold = min_forward_threshold
        self.min_backward_threshold = min_backward_threshold
        self.iteration = 0
        self.best_forward_overall = ((-float('inf'), None), (-float('inf'), None))
        self.best_backward_overall = ((-float('inf'), None), (-float('inf'), None))
        self.best_ratio_overall = ((-float('inf'), None), (-float('inf'), None))
        self.best_composite_overall = ((-float('inf'), None), (-float('inf'), None))
        self.best_forward_meet = ((-float('inf'), None), (-float('inf'), None))
        self.best_backward_meet = ((-float('inf'), None), (-float('inf'), None))
        self.best_ratio_meet = ((-float('inf'), None), (-float('inf'), None))
        self.best_composite_meet = ((-float('inf'), None), (-float('inf'), None))
        self.iterations_data = []  

    @staticmethod
    def update_tracker(tracker, value, candidate):
        best, second = tracker
        if value > best[0]:
            return ((value, candidate), best)
        elif value > second[0]:
            return (best, (value, candidate))
        return tracker

    def compute(self):
        for z_s in self.sun_options:
            for z_p1 in self.p1_options:
                # Using the assumed relationship: z_r1 = z_s + 2*z_p1
                z_r1 = z_s + 2 * z_p1
                for z_p2 in self.p2_options:
                    for z_r2 in self.r2_options:
                        self.iteration += 1
                        gear = Gearbox(z_s, z_p1, z_r1, z_p2, z_r2)
                        gr = gear.gear_ratio()
                        if gr <= 0:
                            continue
                        overall_ratio = 1.0 / gr
                        try:
                            eff_fwd = gear.forward_efficiency()
                            eff_bwd = gear.backward_efficiency()
                        except NotImplementedError:
                            continue
                        if eff_fwd < 0 or eff_bwd < 0:
                            continue
                        if eff_fwd > 2.0 or eff_bwd > 2.0:
                            continue
                        composite_score = overall_ratio * eff_fwd * eff_bwd
                        print(f"Iter #{self.iteration}: Sun:{z_s}, P1:{z_p1}, R1:{z_r1}, "
                              f"P2:{z_p2}, R2:{z_r2} => Gear Ratio: 1:{overall_ratio:.2f}, "
                              f"Fwd: {eff_fwd*100:.2f}%, Bwd: {eff_bwd*100:.2f}%")
                        candidate = (self.iteration, z_s, z_p1, z_r1, z_p2, z_r2,
                                     overall_ratio, eff_fwd, eff_bwd, composite_score)
                        self.iterations_data.append(candidate)
                        self.best_forward_overall = self.update_tracker(self.best_forward_overall, eff_fwd, candidate)
                        self.best_backward_overall = self.update_tracker(self.best_backward_overall, eff_bwd, candidate)
                        self.best_ratio_overall = self.update_tracker(self.best_ratio_overall, overall_ratio, candidate)
                        self.best_composite_overall = self.update_tracker(self.best_composite_overall, composite_score, candidate)
                        if overall_ratio >= self.min_ratio_threshold and eff_fwd >= self.min_forward_threshold and eff_bwd >= self.min_backward_threshold:
                            self.best_forward_meet = self.update_tracker(self.best_forward_meet, eff_fwd, candidate)
                            self.best_backward_meet = self.update_tracker(self.best_backward_meet, eff_bwd, candidate)
                            self.best_ratio_meet = self.update_tracker(self.best_ratio_meet, overall_ratio, candidate)
                            self.best_composite_meet = self.update_tracker(self.best_composite_meet, composite_score, candidate)

    def print_metric(self, metric_name, tracker, threshold):
        best, second = tracker
        if best[0] == -float('inf'):
            print(f"\nNo combination meets {metric_name} threshold of {threshold}.")
            if second[0] != -float('inf'):
                itm, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, fwd, bwd, comp = second[1]
                print(f"Second best overall for {metric_name}: Iteration: {itm} | Sun:{z_s}, "
                      f"P1:{z_p1}, R1:{z_r1}, P2:{z_p2}, R2:{z_r2} => Gear Ratio: 1:{overall_ratio:.2f}, "
                      f"Fwd: {fwd*100:.2f}%, Bwd: {bwd*100:.2f}%, Composite: {comp:.2e}")
            else:
                print(f"No overall combination available for {metric_name}.")
        else:
            itm, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, fwd, bwd, comp = best[1]
            print(f"{metric_name} (req satisfied): Iteration: {itm} | Sun:{z_s}, P1:{z_p1}, R1:{z_r1}, "
                  f"P2:{z_p2}, R2:{z_r2} => Gear Ratio: 1:{overall_ratio:.2f}, Fwd: {fwd*100:.2f}%, "
                  f"Bwd: {bwd*100:.2f}%, Composite: {comp:.2e}")

    def report(self):
        print("\nBest Forward Efficiency:")
        self.print_metric("Forward Efficiency", self.best_forward_meet, self.min_forward_threshold)
        print("\nBest Backward Efficiency:")
        self.print_metric("Backward Efficiency", self.best_backward_meet, self.min_backward_threshold)
        print("\nBest Gear Ratio:")
        self.print_metric("Gear Ratio", self.best_ratio_meet, self.min_ratio_threshold)
        print("\nBest Overall Combination (Composite Score):")
        self.print_metric("Composite Score", self.best_composite_meet, "N/A")

def create_gear(z_teeth, pitch_radius, depth_frac, color):
    depth = pitch_radius * depth_frac
    tip_radius = pitch_radius + depth
    tooth_angle = 2 * np.pi / z_teeth
    points = []
    for t in range(z_teeth):
        theta = t * tooth_angle - tooth_angle / 2
        points.append((pitch_radius * np.cos(theta), pitch_radius * np.sin(theta)))
        theta_center = t * tooth_angle
        thetas = np.linspace(theta_center - 0.2 * tooth_angle, theta_center + 0.2 * tooth_angle, 5)
        for theta in thetas:
            x = tip_radius * np.cos(theta)
            y = tip_radius * np.sin(theta)
            points.append((x, y))
    return Polygon(points, closed=True, fill=False, edgecolor=color, linewidth=1)

def plot_gearbox(candidate):
    # candidate: (iteration, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, eff_fwd, eff_bwd, composite_score)
    _, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, eff_fwd, eff_bwd, _ = candidate
    r_sun = 0.5 * MODULE * z_s
    r_p1 = 0.5 * MODULE * z_p1
    r_r1 = r_sun + 2 * r_p1  # based on your geometry

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    sun = create_gear(z_s, r_sun, 0.15, '#FFD700')
    ax.add_patch(sun)
    ring = create_gear(z_r1, r_r1, 0.1, '#FF0000')
    ring.set_linestyle('--')
    ax.add_patch(ring)
    planet_color = '#0000FF'
    angle_step = 2 * np.pi / N_PLANETS
    for i in range(N_PLANETS):
        theta = i * angle_step
        cx = (r_sun + r_p1) * np.cos(theta)
        cy = (r_sun + r_p1) * np.sin(theta)
        planet = create_gear(z_p1, r_p1, 0.12, planet_color)
        planet.set_transform(Affine2D().translate(cx, cy) + ax.transData)
        ax.add_patch(planet)
    max_dim = r_r1 * 1.2
    ax.set_xlim(-max_dim, max_dim)
    ax.set_ylim(-max_dim, max_dim)
    ax.axis('off')
    
    info_text = (
        f"Best Overall 3K Planetary Gearbox\n"
        f"Sun Gear Teeth: {z_s}\n"
        f"Planet Gear Teeth: {z_p1}\n"
        f"Ring Gear 1 Teeth: {z_r1}\n"
        f"Planet Gear 2 Teeth: {z_p2}\n"
        f"Ring Gear 2 Teeth: {z_r2}\n"
        f"Module: {MODULE}\n"
        f"Friction Coefficient: {MU}\n"
        f"Number of Planets: {N_PLANETS}\n"
        f"Gear Ratio: 1:{1/overall_ratio:.1f}\n"
        f"Forward Efficiency: {eff_fwd*100:.1f}%\n"
        f"Backward Efficiency: {eff_bwd*100:.1f}%\n"
    )
    
    plt.text(0.05, 0.95, info_text, transform=fig.transFigure,
             ha='left', va='top', fontsize=10,
             bbox=dict(facecolor='white', alpha=0.8))
    
    plt.show()

def plot_efficiencies(data, total_iterations, slice_flag=False, target_ratio=None, tolerance=0):
    """
    Creates a 3D scatter plot of overall gear ratio (x), forward efficiency (y), 
    and backward efficiency (z). If slice_flag is True and target_ratio is given,
    the function will plot candidates whose gear ratio falls within the range:
       [target_ratio*(1 - tolerance/100), target_ratio*(1 + tolerance/100)]
    with full opacity (and annotation box colored light green) while other points
    are plotted with low opacity (and annotation box colored light red).
    Only the most recent annotation is displayed.
    """
    import warnings
    warnings.filterwarnings("ignore", message="3d coordinates not supported yet", category=UserWarning)
    
    filtered = [d for d in data if d[7] > 0 and d[8] > 0]
    if not filtered:
        print("No valid data to plot (all filtered).")
        return

    if slice_flag and target_ratio is not None:
        if tolerance == 0:
            lower_bound = target_ratio
            upper_bound = target_ratio
        else:
            lower_bound = target_ratio * (1 - tolerance / 100.0)
            upper_bound = target_ratio * (1 + tolerance / 100.0)
    else:
        lower_bound = None
        upper_bound = None

    data_in = []
    data_out = []
    labels_in = []
    labels_out = []
    gear_ratios_in = []
    gear_ratios_out = []
    fwd_eff_in = []
    fwd_eff_out = []
    bwd_eff_in = []
    bwd_eff_out = []

    for d in filtered:
        itr, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, eff_fwd, eff_bwd, _ = d
        label = (f"Iteration: {itr}\n"
                 f"Sun: {z_s}, P1: {z_p1}, R1: {z_r1}\n"
                 f"P2: {z_p2}, R2: {z_r2}\n"
                 f"Gear Ratio: 1:{overall_ratio:.2f}\n"
                 f"Fwd: {eff_fwd*100:.2f}%, Bwd: {eff_bwd*100:.2f}%")
        if lower_bound is not None and (overall_ratio < lower_bound or overall_ratio > upper_bound):
            data_out.append(d)
            gear_ratios_out.append(overall_ratio)
            fwd_eff_out.append(eff_fwd * 100)
            bwd_eff_out.append(eff_bwd * 100)
            labels_out.append(label)
        else:
            data_in.append(d)
            gear_ratios_in.append(overall_ratio)
            fwd_eff_in.append(eff_fwd * 100)
            bwd_eff_in.append(eff_bwd * 100)
            labels_in.append(label)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    sc_in = None
    if gear_ratios_in:
        sc_in = ax.scatter(gear_ratios_in, fwd_eff_in, bwd_eff_in, 
                         c=fwd_eff_in, cmap='viridis', edgecolor='k', alpha=1, s=50)
    sc_out = None
    if gear_ratios_out:
        sc_out = ax.scatter(gear_ratios_out, fwd_eff_out, bwd_eff_out, 
                         c=fwd_eff_out, cmap='viridis', edgecolor='k', alpha=0.1, s=30)
    
    ax.set_xlabel("Gear Ratio (1:ratio)")
    ax.set_ylabel("Forward Efficiency (%)")
    ax.set_zlabel("Backward Efficiency (%)")
    ax.set_title("Wolfrom Analysis")
    
    global_text = (
        f"Total Iterations: {total_iterations}\n"
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
    cursor = mplcursors.cursor([sc_in, sc_out] if (sc_in is not None and sc_out is not None)
                               else (sc_in if sc_in is not None else sc_out),
                               hover=True, multiple=False)
    
    def on_add(sel):
        sel.annotation.set_visible(True)
        if sel.artist == sc_in:
            sel.annotation.get_bbox_patch().set_facecolor("lightgreen")
            sel.annotation.set_text(labels_in[sel.index])
        else:
            sel.annotation.get_bbox_patch().set_facecolor("lightcoral")
            sel.annotation.set_text(labels_out[sel.index])
    cursor.connect("add", on_add)
    
    fig.colorbar(sc_in if sc_in is not None else sc_out, label="Forward Efficiency (%)")
    plt.show()

def compute_gearbox():
    """
    Compute the 3K Planetary Gearbox by iterating over design options,
    then report metrics and plot results. The slice feature is applied to the
    3D efficiency plot if enabled.
    """
    start_time = time.time()
    options = generate_all_options(SUN_LIMITS, P1_LIMITS, P2_LIMITS, R2_LIMITS, STEPS)
    sun_options = options["sun"]
    p1_options  = options["p1"]
    p2_options  = options["p2"]
    r2_options  = options["r2"]
    
    gear_analysis = GearAnalysis(sun_options, p1_options, p2_options, r2_options)
    gear_analysis.compute()
    print("\nParameters:")
    print(f"Module: {MODULE}, Friction Coefficient: {MU}, Number of Planets: {N_PLANETS}")
    gear_analysis.report()
    
    best_candidate = gear_analysis.best_composite_meet[0][1]
    if best_candidate is None:
        print("No candidate satisfies the thresholds. Skipping plotting.")
    else:
        if DISPLAY_BEST_GEARBOX:
            print("\nPlotting best overall candidate...")
            plot_gearbox(best_candidate)
    
    if DISPLAY_ITERATIONS_PLOT and gear_analysis.iterations_data:
        end_time = time.time() 
        total_time = end_time - start_time
        print(f"Total computation time: {total_time:.2f} seconds")
        print("\nPlotting efficiency results...")
        plot_efficiencies(
            gear_analysis.iterations_data,
            gear_analysis.iteration,
            slice_flag=SLICE_GEAR_RATIO,
            target_ratio=TARGET_GEAR_RATIO,
            tolerance=RATIO_TOLERANCE
        )

def generate_options(lower: int, upper: int, step: int = 1):
    return list(range(lower, upper + 1, step))

def generate_all_options(sun_limits: tuple, p1_limits: tuple, p2_limits: tuple, r2_limits: tuple, 
                         steps: tuple = (1,1,1,1)):
    """
    Generate design options for each gear type.
    Returns a dictionary with keys: "sun", "p1", "p2", "r2".
    """
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