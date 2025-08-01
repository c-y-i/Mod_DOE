# Mod_DOE
Design of experiments for Modlab design projects.

## **Description of the file structure**:

* Root files include .pkl files of tests for design paramaters chose by taguchi array (Taguchi_test_dict, taguchi_valve_dict) and Bayesian Optimization (BayzOp_test_dict, BO_valve_dict_High_lglg). It also includes the optimizations (BayZop_Running & BayZop_Valve) with associated utility functions (utils). It also includes the experimental scripts (Run_Gear_Experiment, Run_Valve_Experiment) with associated utility functions (Experiment_Utils). Finally, it includes .csv files of design parameters for Taguchi generated (Tag_d_params, valve_tag_d_params) and BO generated (d_params, valve_BO_d_params).

* BO_desings includes BO output design values for gear testing.

* Valve_designs includes BO output design values for valve testing.

* wolfrom includes theoretical gear design files used for parameter selection.

## Files and variables:

### **root**:

* BayZop_Running.ipynb: use Ax.dev to run Bayesian Optimization for gear parameter optimization.

* BayzOp_test_dict.pkl: 'pickle' file containing a dictionary of characterization test data for gear designs generated via Bayesian Optimization. Each 'key' contains design number: 'BO_##'. Each 'value' contains a list of [n] elements where each element represents a test. Each test element is a list of [2] elements, where each element represents a data stream. Each data element is a [m_i,2] vector in list form, where the first column represents time [ms] and the second column represents: dynamixel MX-28 torque activation [Element 0], current (mA) [Element 1]

* BayZop_Valve.ipynb: use Ax.dev to run Bayesian Optimization for valve parameter optimization.

* BO_valve_dict_High_lglg.pkl: 'pickle' file containing a dictionary of characterization test data for valve designs generated via Bayesian Optimization. Each 'key' contains information about the pressure level and pneumatic capcitor size, as well as design number: 'Prelim_High_lglg#'. Each 'value' contains a list of [n] elements where each element represents a test. Each test element is a list of [2] elements, where the first element is a [m,1] vector of time values [ms] and the second element is a list corresponding to a [2,m] vector of pressure values [kPa] where the first column represents pressure in the pre-valve pneumatic capacitor and the second column represents pressure in the post-valve pneumatic capacitor.

* d_params.csv: a comma-seperated-value file containing values relevant to CAD-work for gear designs generated via L32b Taguchi array. Wolfram gear design values provided are, in order: design number, number of teeth on sun gear, number of teeth on first part of planet gears, number of teeth on second part of planet gears, number of teeth on ring gear 2, profile shift coefficient for the sun gear, profile shift coefficient for the first part of planet gears, profile shift coefficient for the second part of planet gears, profile shift coefficient for ring gear 2, carrier radii [mm], gear thickness for all gears [mm].

* Experiment_Utils.py: utility functions to aid in valve / gear test experiments.

* Figure_Plotting.ipynb: plots and data analysis relevant to paper results.

* requirements.txt: a list of Python packages required for this code-base.

* Run_Gear_Experiment: functions for user-setup and activated of automated gear testing / data collection.

* Run_Valve_Experiment: functions for user-setup and activated of automated valve testing / data collection.

* Serial_Comms.py: utility functions for serial communication with dynamixel and ESP32.

* Tag_d_params.csv: comma-seperated-value file containing design parameter values and gear ratios for gear designs generated via L32b Taguchi array. Wolfram gear design values provided are, in order: design number, 3d-printer filament, lubricant, half the number of teeth on sun gear, number of teeth on ring gear 2, profile shift coefficient for the sun gear, profile shift coefficient for ring gear 2, clearance between teeth and roots of meshing gears [m], half the gear thickness for all gears [mm], offset integer affecting teeth and profile shift coefficient for planet 1, offset integer affecting teeth and profile shift coefficient  for planet 2, gear ratio of wolfram bilateral planetary gear.

* Taguchi_test_dict.pkl: 'pickle' file containing a dictionary of characterization test data for gear designs generated via L32b Taguchi array. Each 'key' contains design number: 'Taguchi_Design_#'. Each 'value' contains a list of [n] elements where each element represents a test. Each test element is a list of [2] elements, where each element represents a data stream. Each data element is a [2xm_i] vector in list form, where the first column represents time [ms] and the second column represents: dynamixel MX-28 torque activation [Element 0], current (mA) [Element 1]

* taguchi_valve_dict.pkl: 'pickle' file containing a dictionary of characterization test data for valve designs generated via L16b Taguchi array. Each 'key' contains information about the pressure level and pneumatic capcitor size, as well as design number: 'Prelim_High_lglg#'. Each 'value' contains a list of [n] elements where each element represents a test. Each test element is a list of [2] elements, where the first element is a [m,1] vector of time values [ms] and the second element is a list corresponding to a[2,m] vector of pressure values [kPa] where the first column represents pressure in the pre-valve pneumatic capacitor and the second column represents pressure in the post-valve pneumatic capacitor.

* utils.py: utility functions to aid in valve / gear analysis and Bayesian Optimization.

* valve_BO_d_params.csv: comma-seperated-value file containing design parameter values and test results for valve designs generated via Bayesian Optimization. Design values provided are, in order: design number, material (Ecoflex/Dragonskin silicone), valve thickness [mm], valve dome height [mm], cut length for slit in silicone valve [mm], cut geometry, test result for crack pressure [kPa], test result for steady-state pressure difference [kPa].

* valve_tag_d_params.csv: comma-seperated-value file containing design parameter values for valve designs generated via L16b Taguchi array. Design values provided are, in order: design number, material (Ecoflex/Dragonskin silicone), valve thickness [mm], valve dome height [mm], cut length for slit in silicone valve [mm], cut geometry, test result for crack pressure [kPa], test result for steady-state pressure difference [kPa].

### **Arduino**:

* 2025_05_20_SparkFun_kPa_Send_Median_Filter: Arduino IDE .ino file for ESP-32 driven pressure monitoring via MPRLS0025 sensor and data transmission via ESP-NOW.

* 2025_05_21_p_receieve_x2_serial_commands: Arduino IDE .ino file for ESP-32 driven receiving of pressure data via ESP-NOW and transfer to Python code via serial communication.

* 2025-03-05-ina260_test: Arduino IDE .ino file for ESP-32 driven current sensing via ina260 sensor with transfer to Python code via serial communication.

* 2025-03-11_LoadCell_Record_N: Arduino IDE .ino file for ESP-32 driven loadcell sensing for friction estimation with transfer to Python code via serial communication.

### **BO_designs**: 

* BO_d_params.csv: a comma-seperated-value file containing values relevant to CAD-work for gear designs generated via Bayesian Optimization. Wolfram gear design values provided are, in order: design number, number of teeth on sun gear, number of teeth on first part of planet gears, number of teeth on second part of planet gears, number of teeth on ring gear 2, profile shift coefficient for the sun gear, profile shift coefficient for the first part of planet gears, profile shift coefficient for the second part of planet gears, profile shift coefficient for ring gear 2, carrier radii [mm], gear thickness for all gears [mm]. 

* BO_OneShot_d_vals.csv: a comma-seperated-value file containing values relevant to CAD-work for gear design generated via Bayesian Optimization from all Taguchi tests. Wolfram gear design values provided are, in order: design number, number of teeth on sun gear, number of teeth on first part of planet gears, number of teeth on second part of planet gears, number of teeth on ring gear 2, profile shift coefficient for the sun gear, profile shift coefficient for the first part of planet gears, profile shift coefficient for the second part of planet gears, profile shift coefficient for ring gear 2, carrier radii [mm], gear thickness for all gears [mm], clearance between teeth and roots of meshing gears [m], 3d-printer filament, lubricant.

* BO_OneShot_meta_vals.csv: comma-seperated-value file containing design parameter values and gear ratios for gear designs generated via Bayesian Optimization from all Taguchi tests. Wolfram gear design values provided are, in order: design number, 3d-printer filament, lubricant, half the number of teeth on sun gear, number of teeth on ring gear 2, profile shift coefficient for the sun gear, profile shift coefficient for ring gear 2, clearance between teeth and roots of meshing gears [m], half the gear thickness for all gears [mm], offset integer affecting teeth and profile shift coefficient for planet 1, offset integer affecting teeth and profile shift coefficient for planet 2, gear ratio of wolfram bilateral planetary gear.

* trial_[XX,YY]_d_vals.csv: comma-seperated-value file containing design parameter values and gear ratios for gear designs generated via Bayesian Optimization during acquisition that yielded design numbers XX and YY. Wolfram gear design values provided are, in order: design number, 3d-printer filament, lubricant, half the number of teeth on sun gear, number of teeth on ring gear 2, profile shift coefficient for the sun gear, profile shift coefficient for ring gear 2, clearance between teeth and roots of meshing gears [m], half the gear thickness for all gears [mm], offset integer affecting teeth and profile shift coefficient for planet 1, offset integer affecting teeth and profile shift coefficient for planet 2, gear ratio of wolfram bilateral planetary gear.

### **Valve_designs**: 

* valve_trial_[X].csv: comma-seperated-value file containing design parameter values for valve designs generated via Bayesian Optimization during acquisition that yielded design number X. Design values provided are, in order: design number, material (Ecoflex/Dragonskin silicone), cut geometry, valve thickness [mm], valve dome height [mm], cut length for slit in silicone valve [mm].


### **wolfrom**

This folder contains tools, scripts, and data for Wolfrom gearbox design and optimization.

* **gear_maker** - Gear Generator add-in for Fusion 360  
  - `gear_maker.py` - Source code for the add-in.  
  - `gear_maker.manifest` - Add-in metadata file required for Fusion 360 to run the script.  

* **config.json** - Parameter definitions and value ranges for gearbox generation.  

* **design_params.py** - Alternative method for generating design parameters for the Taguchi Array.  

* **cross_reference.py** - Post-processes results from `design_params.py` to create candidate designs for the Taguchi Array.  

* **gear_optimization_visualization.ipynb** - Jupyter Notebook for visualizing the gear optimization design space.


### TODO for other files/structure (remove what we can, explain the rest.)

* gear_optimization_visualization.ipynb: design space visualization for choosing relevant gear parameter inputs to L32b taguchi array.

### **CAD Files**

This folder contains printable parts and assemblies for the Wolfrom gearbox (Bayesian Optimization trial **BO22**) and the valve mold set (All-trials One-Shot).

#### [`gear_BO22`](./CAD/gear_BO22) - Wolfrom gearbox (BO trial 22)
Files here provide both individual STL parts for 3D printing and full assemblies for reference/CAD transfer.

* **STL parts (ready to print)**
  - `22_sun_12_0.7513_6.stl` - sun gear  
  - `22_planet_36_0.7854_28_*.stl` - two-stage planet gear (first and second segments)  
  - `22_ring2_76_0.1686_6.stl` - ring gear 2  
  - `ring1_90_6.stl` - ring gear 1  
  - `22_carrier2_20.7_6.stl` - planet carrier  

  *Naming note:* Filenames encode trial and key parameters (e.g., tooth counts, profile shift, thickness in mm).

* **Assemblies**
  - `BO_22_Gearbox.f3d` - Fusion 360 design file for the BO22 assembly.
  - `BO_22_Gearbox.step` - neutral STEP export of the same assembly for use outside Fusion.

#### [`valve_ALL_OS`](./CAD/valve_ALL_OS) - Valve (All-trials One-Shot)
Parametric solids for the “one-shot” valve mold halves:

* `ALL_OS_f.SLDPRT` - **female** (cavity) half.  
* `ALL_OS_m.SLDPRT` - **male** (protruding/core) half.