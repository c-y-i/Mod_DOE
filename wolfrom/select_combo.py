import pandas as pd
from collections import Counter
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, "results", "valid_combinations.csv")
df = pd.read_csv(csv_path, header=None, usecols=range(8))
combinations = df.values.tolist()

k = 4

# Compute frequency counts per parameter.
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
print("Selection per parameter:")
for i, subset in enumerate(best_selection):
    print(f"Parameter {i+1}: {subset}")

print("Number of valid combinations:", count_top4)

valid_rows = [row for row in combinations if all(row[i] in best_selection[i] for i in range(8))]
output_csv_path = os.path.join(script_dir, "results", f"valid_k_{k}.csv")
pd.DataFrame(valid_rows).to_csv(output_csv_path, header=False, index=False)
print(f"Saved {len(valid_rows)} valid combinations to {output_csv_path}")