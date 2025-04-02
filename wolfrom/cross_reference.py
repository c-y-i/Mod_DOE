"""
The csv is generated in clean_design_param.py
For a larger dataset, we can tune the `tol` variable, with the default value of 0.1.
But to be conservative, we should keep tolerance as it is. 
One important parameter that we can tune, is the step size for x in config.json
"""
import pandas as pd
from collections import Counter
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, "results", "valid_combinations.csv")
df = pd.read_csv(csv_path, header=None, usecols=range(8))
combinations = df.values.tolist()

k = 4

best_selection = []
for i in range(8):
    counts = Counter(row[i] for row in combinations)
    top_k = [item for item, freq in counts.most_common(k)]
    best_selection.append(tuple(top_k))

def count_valid(selection):
    count = 0
    for row in combinations:
        if all(row[i] in selection[i] for i in range(8)):
            count += 1
    return count

count_top4 = count_valid(best_selection)
print("Selection per parameter (frequency-based):")
for i, subset in enumerate(best_selection):
    print(f"Parameter {i+1}: {subset}")
print("Result without presets:", count_top4)

valid_rows = [row for row in combinations if all(row[i] in best_selection[i] for i in range(8))]
output_csv_path = os.path.join(script_dir, "results", f"valid_k_{k}.csv")
pd.DataFrame(valid_rows).to_csv(output_csv_path, header=False, index=False)

default_values = [12, 39, 32, 81, 0.5, 0.25, -0.5, -0.25] # table III parameters
best_selection_defaults = []
for i in range(8):
    default_val = default_values[i]
    counts = Counter(row[i] for row in combinations)
    selection = [default_val] # start with default val
    top_candidates = [item for item, freq in counts.most_common() if item != default_val]
    selection += top_candidates[:k-1] 
    best_selection_defaults.append(tuple(selection))

count_top4_defaults = count_valid(best_selection_defaults)
print("\nSelection per parameter with presets:")
for i, subset in enumerate(best_selection_defaults):
    print(f"Parameter {i+1}: {subset}")
print("Result with presets (k**):", count_top4_defaults)

valid_rows_defaults = [row for row in combinations if all(row[i] in best_selection_defaults[i] for i in range(8))]
output_csv_path_defaults = os.path.join(script_dir, "results", f"valid_k_{k}_preset.csv")
pd.DataFrame(valid_rows_defaults).to_csv(output_csv_path_defaults, header=False, index=False)
