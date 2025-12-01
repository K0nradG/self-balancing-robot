import subprocess
from pathlib import Path

EXCLUDE_DIRS = {"build"} 

def format_sources(root: Path):
    # Recursively find .cpp and .h files, excluding certain directories
    files = [
        f for f in root.rglob("*")
        if f.suffix in {".cpp", ".h"} and not any(part in EXCLUDE_DIRS for part in f.parts)
    ]

    if not files:
        print("No .cpp or .h files found.")
        return

    print(f"Formatting all .cpp and .h files under {root}, excluding {EXCLUDE_DIRS}...\n")
    for f in files:
        print(f"Formatting: {f.relative_to(root)}")
        subprocess.run(["clang-format", "-i", str(f)], check=False)

    print("\nFormatting completed!")

if __name__ == "__main__":
    script_folder = Path(__file__).parent.parent.parent
    format_sources(script_folder)