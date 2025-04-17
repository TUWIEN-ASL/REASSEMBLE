import glob
from REASSEMBLE.io import load_h5_file
import tqdm
import os

processed_path = "/home/dsliwowski/Projects/REASSEMBLE/REASSEMBLE/data/REASSEMBLE_corrected/data/*.h5"

file_paths = glob.glob(processed_path)

actions = ["Pick", "Insert", "Remove", "Place"]
objects = ["round peg 1", "round peg 2", "round peg 3", "round peg 4",
           "square peg 1", "square peg 2", "square peg 3", "square peg 4",
           "BNC", "D-SUB", "USB", "Ethernet", "waterproof",
           "bolt 4",
           "large gear", "medium gear", "small gear"]

expected_obj = ["No action."]
for a in actions:
    for o in objects:
        expected_obj.append(a + " " + o + ".")

for path in tqdm.tqdm(file_paths):
    f_name = os.path.split(path)[1]
    try:
        data = load_h5_file(path, decode=False)
    except:
        print("Error reading ", f_name)
        continue

    for k, v in data["segments_info"].items():
        action = v["text"].decode("utf-8")
        if action not in expected_obj:
            print(action)
            print("Unexpected action", path)