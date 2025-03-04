"""
    Analysis of a 3K Planetary Gearbox

    TODO: 
    - Implement epsilon in gearbox.py
    - Support other parameters, from the 2/21 meeting, such as x_i, lubrication, materials...
    - Convert and merge profile shift coefficient function code from Matlab
    - Rework plot gearbox func (optional)
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


# For filtering
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
                # Ring 1 is constrained by Sun and P1, by the following equation
                z_r1 = z_s + 2 * z_p1
                for z_p2 in self.p2_options:
                    for z_r2 in self.r2_options:
                        self.iteration += 1
                        gear = Gearbox(z_s, z_p1, z_r1, z_p2, z_r2)
                        gr = gear.gear_ratio()
                        overall_ratio = 1 / gr if gr != 0 else float('inf')
                        try:
                            eff_fwd = gear.forward_efficiency()
                            eff_bwd = gear.backward_efficiency()
                        except NotImplementedError:
                            continue
                        print(f"Iter #{self.iteration}: Sun:{z_s}, P1:{z_p1}, R1:{z_r1}, "
                              f"P2:{z_p2}, R2:{z_r2} => Gear Ratio: 1:{overall_ratio:.2f}, "
                              f"Fwd: {eff_fwd*100:.2f}%, Bwd: {eff_bwd*100:.2f}%")
                        composite_score = overall_ratio * eff_fwd * eff_bwd
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
    # candidate format:
    # (iteration, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, eff_fwd, eff_bwd, composite_score)
    _, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, eff_fwd, eff_bwd, _ = candidate
    r_sun = 0.5 * MODULE * z_s
    r_p1 = 0.5 * MODULE * z_p1
    r_r1 = r_sun + 2 * r_p1
    
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

def plot_efficiencies(data, total_iterations):
    import warnings
    warnings.filterwarnings("ignore", message="3d coordinates not supported yet", category=UserWarning)
    filtered = [d for d in data if d[7] > 0 and d[8] > 0] 
    gear_ratios = [d[6] for d in filtered]  # overall_ratio
    fwd_eff = [d[7]*100 for d in filtered]  # forward efficiency (%)
    bwd_eff = [d[8]*100 for d in filtered]  # backward efficiency (%)
    labels = []
    for d in filtered:
        # d = (iteration, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, eff_fwd, eff_bwd, composite_score)
        itr, z_s, z_p1, z_r1, z_p2, z_r2, overall_ratio, eff_fwd, eff_bwd, _ = d
        label = (f"Iteration: {itr}\n"
                 f"Sun: {z_s}, P1: {z_p1}, R1: {z_r1}\n"
                 f"P2: {z_p2}, R2: {z_r2}\n"
                 f"Gear Ratio: 1:{overall_ratio:.2f}\n"
                 f"Fwd: {eff_fwd*100:.2f}%, Bwd: {eff_bwd*100:.2f}%")
        labels.append(label)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(gear_ratios, fwd_eff, bwd_eff, c=fwd_eff, cmap='viridis', edgecolor='k')
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
        f"Step Sizes: {STEPS}"
    )
    anchored_text = AnchoredText(
        global_text,
        loc='upper left',
        bbox_to_anchor=(0, 1),
        bbox_transform=plt.gcf().transFigure,
        prop=dict(size=10),
        frameon=True
    )
    ax.add_artist(anchored_text)
    import matplotlib as mpl
    norm = mpl.colors.Normalize(vmin=min(fwd_eff), vmax=max(fwd_eff))
    cmap = plt.get_cmap('viridis')
    cursor = mplcursors.cursor(sc, hover=True)
    def on_add(sel):
        sel.annotation.arrow_patch.set_visible(False)
        sel.annotation.set_text(labels[sel.index])
        point_val = fwd_eff[sel.index]
        color = cmap(norm(point_val))
        sel.annotation.get_bbox_patch().set_facecolor(color)
    cursor.connect("add", on_add)
    
    fig.colorbar(sc, label="Forward Efficiency (%)")
    plt.show()

def compute_gearbox():
    """
    compute the 3K Planetary Gearbox by iterating over design options.
    Option selection is now integrated with generated options.
    Limits and step sizes for each gear are provided to generate_all_options.
    """
    # limits and steps for each gear type.
    # order: sun, p1, p2, r2, steps
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
        if DISPLAY_BEST_GEARBOX: # if flag is True...
            print("\nPlotting best overall candidate...")
            plot_gearbox(best_candidate)
    # ONLY plot if best candidate is found and flag is True.
    if DISPLAY_ITERATIONS_PLOT and best_candidate is not None and gear_analysis.iterations_data:
        end_time = time.time() 
        total_time = end_time - start_time
        print(f"Total computation time: {total_time:.2f} seconds")
        print("\nPlotting efficiency results...")
        plot_efficiencies(gear_analysis.iterations_data, gear_analysis.iteration)  # pass total iterations


def generate_options(lower: int, upper: int, step: int = 1):
    # Helper function
    return list(range(lower, upper + 1, step))

def generate_all_options(sun_limits: tuple, p1_limits: tuple, p2_limits: tuple, r2_limits: tuple, 
                         steps: tuple = (1,1,1,1)):
    """
    Parameters:
    - sun_limits: (lower, upper) limits for the sun gear teeth count.
    - p1_limits: (lower, upper) limits for the planet gear 1 teeth count.
    - p2_limits: (lower, upper) limits for the planet gear 2 teeth count.
    - r2_limits: (lower, upper) limits for the ring gear 2 teeth count.
    - steps: Tuple of steps (step_sun, step_p1, step_p2, step_r2). Default is (1,1,1,1).
    """
    step_sun, step_p1, step_p2, step_r2 = steps
    options = {
        "sun": generate_options(sun_limits[0], sun_limits[1], step_sun),
        "p1": generate_options(p1_limits[0], p1_limits[1], step_p1),
        "p2": generate_options(p2_limits[0], p2_limits[1], step_p2),
        "r2": generate_options(r2_limits[0], r2_limits[1], step_r2)
    }
    return options


if __name__ == "__main__":
    print(f"Starting computation based on defined parameters...\n")
    compute_gearbox()
