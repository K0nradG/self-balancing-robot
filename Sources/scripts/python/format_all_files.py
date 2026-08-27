# Copyright 2026 Filip Dymczyk and Konrad Grucel

import subprocess
from pathlib import Path

EXCLUDE_DIRS = {"build", "3rdParty"}


def format_sources(root: Path):
    # Recursively find .cpp, .h and .c files, excluding certain directories
    files = [
        f
        for f in root.rglob("*")
        if f.suffix in {".cpp", ".h", ".c"}
        and not any(part in EXCLUDE_DIRS for part in f.parts)
    ]

    if not files:
        print
        ("No .cpp, .h or .c files found.")
        return

    print(
        f"Formatting all .cpp, .h and .c files under {root}, excluding {EXCLUDE_DIRS}...\n"
    )
    for f in files:
        print(f"Formatting: {f.relative_to(root)}")
        subprocess.run(["clang-format", "-i", str(f)], check=False)

    print("\nFormatting completed!")


if __name__ == "__main__":
    script_folder = Path(__file__).parent.parent.parent
    format_sources(script_folder)
