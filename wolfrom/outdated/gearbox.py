import numpy as np
import json
import os

# Load configuration
config_path = os.path.join(os.path.dirname(__file__), "config.json")
with open(config_path, "r") as f:
    config = json.load(f)

MODULE = config["MODULE"]
MC = config.get("MC", MODULE)
MU = config["MU"]
ALPHA = np.radians(config.get("ALPHA", 20))
XS = config.get("XS", 0.0)
XP1 = config.get("XP1", 0.0)
XP2 = config.get("XP2", 0.0)
XR1 = config.get("XR1", 0.0)
XR2 = config.get("XR2", 0.0)

class Gearbox:
    def __init__(self, z_s, z_p1, z_r1, z_p2, z_r2,
                 module=MODULE, mc=MC, mu=MU, alpha=ALPHA,
                 xs=XS, xp1=XP1, xp2=XP2, xr1=XR1, xr2=XR2):
        self.z_s = z_s
        self.z_p1 = z_p1
        self.z_r1 = z_r1
        self.z_p2 = z_p2
        self.z_r2 = z_r2
        self.ma = module  
        self.mc = mc
        self.mu = mu
        self.alpha = alpha
        self.xs = xs
        self.xp1 = xp1
        self.xp2 = xp2
        self.xr1 = xr1
        self.xr2 = xr2

    def gear_ratio(self):
        I1 = self.z_r1 / self.z_s
        I2 = (self.z_r1 * self.z_p2) / (self.z_p1 * self.z_r2)
        return (1 - I2) / (1 + I1)

    def _compute_efficiencies(self):
        # Compute gear ratio terms
        I1 = self.z_r1 / self.z_s
        I2 = (self.z_r1 * self.z_p2) / (self.z_p1 * self.z_r2)
        ra = (self.z_s * self.ma + self.z_p1 * self.ma) / 2.0
        # Basic circle diameters
        db_a1 = self.ma * self.z_s * np.cos(self.alpha)
        db_a2 = self.ma * self.z_p1 * np.cos(self.alpha)
        db_b1 = self.ma * self.z_p1 * np.cos(self.alpha)
        db_b2 = self.ma * self.z_r1 * np.cos(self.alpha)
        db_c1 = self.mc * self.z_p2 * np.cos(self.alpha)
        db_c2 = self.mc * self.z_r2 * np.cos(self.alpha)
        # Working pressure angles (modified formula)
        alpha_wa = np.arccos(self.ma * (self.z_s + self.z_p1) * np.cos(self.alpha) / (2 * ra))
        alpha_wb = np.arccos(self.ma * (-self.z_p1 + self.z_r1) * np.cos(self.alpha) / (2 * ra))
        val = self.mc * (-self.z_p2 + self.z_r2) * np.cos(self.alpha) / (2 * ra)
        alpha_wc = np.arccos(np.clip(val, -1, 1))
        # Center distance modification coefficient
        ya = ((self.z_s + self.z_p1) / 2.0) * ((np.cos(self.alpha) / np.cos(alpha_wa)) - 1)
        # Tip circle diameters
        das  = self.ma * self.z_s + 2 * self.ma * (1 + ya - self.xp1)
        dap1 = self.ma * self.z_p1 + 2 * self.ma * (1 + min(ya - self.xs, self.xp1))
        dar1 = self.ma * self.z_r1 - 2 * self.ma * (1 - self.xr1)
        dap2 = self.mc * self.z_p2 + 2 * self.mc * (1 + self.xp2)
        dar2 = self.mc * self.z_r2 - 2 * self.mc * (1 - self.xr2)
        # Tip pressure angles
        alpha_aa1 = np.arccos(db_a1 / das)
        alpha_aa2 = np.arccos(db_a2 / dap1)
        alpha_ab1 = np.arccos(db_b1 / dap1)
        alpha_ab2 = np.arccos(db_b2 / dar1)
        alpha_ac1 = np.arccos(db_c1 / dap2)
        val2 = db_c2 / dar2
        alpha_ac2 = np.arccos(np.clip(val2, -1, 1))
        # Approach and recess contact ratios
        ea1 = (self.z_p1 / (2 * np.pi)) * (np.tan(alpha_aa2) - np.tan(alpha_wa))
        ea2 = (self.z_s / (2 * np.pi)) * (np.tan(alpha_aa1) - np.tan(alpha_wa))
        eb1 = -(self.z_r1 / (2 * np.pi)) * (np.tan(alpha_ab2) - np.tan(alpha_wb))
        eb2 = (self.z_p1 / (2 * np.pi)) * (np.tan(alpha_ab1) - np.tan(alpha_wb))
        ec1 = -(self.z_r2 / (2 * np.pi)) * (np.tan(alpha_ac2) - np.tan(alpha_wc))
        ec2 = (self.z_p2 / (2 * np.pi)) * (np.tan(alpha_ac1) - np.tan(alpha_wc))
        ea = ea1**2 + ea2**2 - ea1 - ea2 + 1
        eb = eb1**2 + eb2**2 - eb1 - eb2 + 1
        ec = ec1**2 + ec2**2 - ec1 - ec2 + 1
        # Basic driving efficiencies
        Ea_val = 1 - self.mu * np.pi * (1 / self.z_s + 1 / self.z_p1) * ea
        Eb_val = 1 - self.mu * np.pi * (1 / self.z_p1 - 1 / self.z_r1) * eb
        Ec_val = 1 - self.mu * np.pi * (1 / self.z_p2 - 1 / self.z_r2) * ec
        # Forward driving efficiency 
        eta_fwd = (1 + Ea_val * Eb_val * I1) * (1 - I2) / ((1 + I1) * (1 - Eb_val * Ec_val * I2))
        # Backward driving efficiency
        eta_bwd = (1 + I1) * Ea_val * (Eb_val * Ec_val - I2) / (Ec_val * (Ea_val * Eb_val + I1) * (1 - I2))
        return {
            "I1": I1,
            "I2": I2,
            "Ea": Ea_val,
            "Eb": Eb_val,
            "Ec": Ec_val,
            "eta_fwd": eta_fwd,
            "eta_bwd": eta_bwd
        }

    def forward_efficiency(self):
        eff = self._compute_efficiencies()
        return eff["eta_fwd"]

    def backward_efficiency(self):
        eff = self._compute_efficiencies()
        return eff["eta_bwd"]

def test():
    Z_SUN = 12 # Sun gear teeth (Table III)
    Z_P1 = 39 # Planet gear 1 teeth (Table III)
    Z_R1 = 90 # Ring gear 1 teeth (Table III)
    Z_P2 = 32 # Planet gear 2 teeth (Table III)
    Z_R2 = 81 # Ring gear 2 teeth (Table III)
    
    # Instantiate Gearbox instance using config for the rest of parameters
    gearbox = Gearbox(Z_SUN, Z_P1, Z_R1, Z_P2, Z_R2)
    
    gr = gearbox.gear_ratio()
    eta_fwd = gearbox.forward_efficiency()
    eta_bwd = gearbox.backward_efficiency()
    
    print("Gear Ratio:", 1/gr)
    print("Forward Efficiency:", eta_fwd)
    print("Backward Efficiency:", eta_bwd)

# if __name__ == "__main__":
#     test()

