# Copyright 2026 Filip Dymczyk and Konrad Grucel

from pathlib import Path
import subprocess

# Local helper:
def get_changed_files():
    """Return a list of changed or staged .cpp/.c/.h/.hpp files in the git repo."""
    PROJECT_DIR = Path(__file__).resolve().parents[2]  # Sources/scripts/python -> Sources
    try:
        result = subprocess.run(
            ["git", "diff", "--name-only", "--diff-filter=ACM", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
            cwd=PROJECT_DIR
        )
        files = []
        for f in result.stdout.splitlines():
            if f.endswith((".cpp", ".c", ".h", ".hpp")):
                 # Resolve the path relative to PROJECT_DIR parent, otherwise their would be duplication of path content.
                full_path = PROJECT_DIR.parent / f
                if full_path.exists():
                    files.append(full_path)
                else:
                    print(f"Warning: file {full_path} does not exist, skipping.")
        return files
    except subprocess.CalledProcessError as e:
        print("Error running git:", e)
        return []

def format_changed_files():
    files: list[Path] = get_changed_files()
    if not files:
        print("No changed .cpp, .c, .h, or .hpp files detected.")
        return

    print("Formatting changed files...")
    for f in files:
        print(f"Formatting:", f.name)
        subprocess.run(["clang-format", "-i", str(f)], check=False)

    print("Formatting completed!")
