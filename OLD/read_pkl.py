import pickle
import os
import csv
import json

def read_pkl(file_path):
    """
    Reads a pickle file and returns the data.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"The file {file_path} does not exist.")
    
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    
    return data

def save_to_csv(data, output_file):
    """
    Saves data to a CSV file.
    """
    if isinstance(data, dict):
        # If every value is a list/tuple, treat as dict-of-lists:
        if all(isinstance(v, (list, tuple)) for v in data.values()):
            keys = list(data.keys())
            rows = zip(*data.values())
            with open(output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(keys)
                writer.writerows(rows)
            return
        data = [data]  # Wrap dict (not lists) in a list for CSV writing
    
    keys = data[0].keys() if isinstance(data[0], dict) else range(len(data[0]))
    with open(output_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys) if isinstance(data[0], dict) else csv.writer(f)
        writer.writeheader() if isinstance(data[0], dict) else None
        writer.writerows(data if isinstance(data[0], dict) else data)

def save_to_json(data, output_file):
    """
    Saves data to a JSON file.
    """
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=4)

if __name__ == "__main__":
    file_path = 'valve_dict2.pkl'
    try:
        data = read_pkl(file_path)
        print("Data loaded successfully.")
        print(data)
        
        # csv_file = 'output_data.csv'
        # save_to_csv(data, csv_file)
        # print(f"Data saved to CSV: {csv_file}")
        
        # Save to JSON
        json_file = 'output_data.json'
        save_to_json(data, json_file)
        print(f"Data saved to JSON: {json_file}")
    except Exception as e:
        print(f"An error occurred: {e}")