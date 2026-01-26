import pandas as pd
import glob
import os

# Katalog roboczy (bieżący)
DATA_DIR = "."

# Wyszukanie plików ident_*.csv
files = sorted(glob.glob(os.path.join(DATA_DIR, "ident_*.csv")))

if not files:
    raise FileNotFoundError("Nie znaleziono plików ident_*.csv")

dfs = []

for file in files:
    df = pd.read_csv(file)

    # opcjonalnie: usunięcie timestamp
    if "timestamp" in df.columns:
        df = df.drop(columns=["timestamp"])

    # dodanie informacji z którego pliku pochodzi wiersz
    df["source_file"] = os.path.basename(file)

    dfs.append(df)

# Scalenie
merged_df = pd.concat(dfs, ignore_index=True)

# Zapis
merged_df.to_csv("merged.csv", index=False)

print(f"Scalono {len(files)} plików -> merged.csv")

