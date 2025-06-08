import pandas as pd

front = "../simulation/logs"
_type = "mpc"
files = [
    f"{front}/{_type}1.log",
    f"{front}/{_type}2.log",
    f"{front}/{_type}3.log",
]

dfs = []
for i, fp in enumerate(files, start=1):
    df = pd.read_csv(
        fp,
        header=0,  # ← skip the original header row
        names=[
            "timestamp_s",
            f"pitch_degrees_t{i}",
            f"control_latency_microseconds_t{i}",
        ],
        usecols=[0, 1, 2],  # ← read timestamp, pitch, latency
        dtype=float,
    )
    dfs.append(df)

# outer‐join on timestamp_s
merged = dfs[0]
for df in dfs[1:]:
    merged = merged.merge(df, on="timestamp_s", how="outer")

# sort by time
merged = merged.sort_values("timestamp_s")

# write out
merged.to_csv(f"{front}/combined_trials.csv", index=False)
print("combined_trials.csv written.")
