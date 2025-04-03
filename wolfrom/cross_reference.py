"""
Cleaned up version of the cross_reference
Output the candidates for the Taguchi Array
"""

import pandas as pd
from collections import Counter
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, "results", "valid_combinations.csv")
output_csv = os.path.join(script_dir, "results", "filtered_valid_rows.csv")
n = 8
k = 4
column_names = ["z_s", "z_p1", "z_p2", "z_r2", "x_s", "x_p1", "x_p2", "x_r2"]

def filter(df):
    candidate_lists = []
    for col in range(n):
        counts = Counter(df[col])
        top_values = [item for item, _ in counts.most_common(k)]
        candidate_lists.append(top_values)
    print("\nParameter Lists:")
    for col in range(n):
        print(f"{column_names[col]}: {candidate_lists[col]}")
    # Step 2: Filter rows that match the candidate lists
    filtered_rows = df[df.apply(lambda row: all(row[col] in candidate_lists[col] for col in range(n)), axis=1)].copy()
    # Ensure all candidate values are covered
    for col in range(n):
        missing_values = set(candidate_lists[col]) - set(filtered_rows[col].unique())
        if missing_values:
            additional_rows = df[df[col].isin(missing_values)]
            filtered_rows = pd.concat([filtered_rows, additional_rows]).drop_duplicates().reset_index(drop=True)
    print("\nAppearances in Filtered Results:")
    for col in range(n):
        value_counts = Counter(filtered_rows[col])
        print(f"{column_names[col]}: " + ", ".join(f"{value} ({value_counts[value]})" for value in candidate_lists[col]))
    filtered_rows.to_csv(output_csv, index=False, header=False)


if __name__ == "__main__":
    # Read the input CSV file
    df = pd.read_csv(csv_path, header=None, usecols=range(n))
    filter(df)