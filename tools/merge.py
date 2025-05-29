import pandas as pd

front = "../simulation/logs/pid"
files = [
    f"{front}/trial_1.log",
    f"{front}/trial_2.log",
    f"{front}/trial_3.log",
]

dfs = []
for i, fp in enumerate(files, start=1):
    df = pd.read_csv(
        fp,
        header=0,  # ← the FIRST row is the header
        names=["timestamp_seconds", f"trial_{i}_pitch_deg"],  # ← override them
        usecols=[0, 1],  # ← only read those two cols
        dtype=float,  # ← cast all data rows to float
    )
    dfs.append(df)

# outer‐join on the timestamp
merged = dfs[0]
for df in dfs[1:]:
    merged = merged.merge(df, on="timestamp_seconds", how="outer")

# make sure it’s in ascending time order
merged = merged.sort_values("timestamp_seconds")

merged.to_csv(f"{front}/combined_trials.csv", index=False)
print("combined_trials.csv written.")
