# TODO: Implement epsilon

import numpy as np
import json
import os

config_path = os.path.join(os.path.dirname(__file__), "config.json")
with open(config_path, "r") as f:
    config = json.load(f)

MODULE = config["MODULE"]
MU = config["MU"]

class Gearbox:
    def __init__(self, z_s, z_p1, z_r1, z_p2, z_r2, module=MODULE, mu=MU):
        self.z_s = z_s
        self.z_p1 = z_p1
        self.z_r1 = z_r1
        self.z_p2 = z_p2
        self.z_r2 = z_r2
        self.module = module
        self.mu = mu

    @staticmethod
    def basic_efficiency(z1, z2, mu, sign=1):
        """
        TODO: Implement epsilon
        Eq 78, 79, 80 in the paper
        
        return: 1 - mu * np.pi * (1/z1 + sign/z2) * e_i
        """
        return 1 - mu * np.pi * (1/z1 + sign/z2)

    def gear_ratio(self):
        I1 = self.z_r1 / self.z_s
        I2 = (self.z_r1 * self.z_p2) / (self.z_r2 * self.z_p1)
        return (1 - I2) / (1 + I1)

    def forward_efficiency(self):
        I1 = self.z_r1 / self.z_s
        I2 = (self.z_r1 * self.z_p2) / (self.z_r2 * self.z_p1)
        eff_a = Gearbox.basic_efficiency(self.z_s, self.z_p1, self.mu, sign=1)
        eff_b = Gearbox.basic_efficiency(self.z_p1, self.z_r1, self.mu, sign=-1)
        eff_c = Gearbox.basic_efficiency(self.z_p2, self.z_r2, self.mu, sign=-1)
        if I2 < 1:
            return (1 + eff_a * eff_b * I1) * (1 - I2) / ((1 + I1) * (1 - eff_b * eff_c * I2))
        else:
            return eff_c * (eff_b + eff_a * I1) * (1 - I2) / ((1 + I1) * (eff_b * eff_c - I2))

    def backward_efficiency(self):
        I1 = self.z_r1 / self.z_s
        I2 = (self.z_r1 * self.z_p2) / (self.z_r2 * self.z_p1)
        eff_a = Gearbox.basic_efficiency(self.z_s, self.z_p1, self.mu, sign=1)
        eff_b = Gearbox.basic_efficiency(self.z_p1, self.z_r1, self.mu, sign=-1)
        eff_c = Gearbox.basic_efficiency(self.z_p2, self.z_r2, self.mu, sign=-1)
        if I2 < 1:
            return (1 + I1) * eff_a * (eff_b * eff_c - I2) / (eff_c * (eff_a * eff_b + I1) * (1 - I2))
        else:
            raise NotImplementedError("I2 > 1 not implemented") # Zero