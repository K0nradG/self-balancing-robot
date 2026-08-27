# Copyright 2026 Filip Dymczyk and Konrad Grucel

from pathlib import Path

EXCLUDE_DIRS = {"build", ".git", "__pycache__"}
EXTENSIONS = {
    ".c",
    ".cpp",
    ".h",
    ".hpp",
    ".py",
    ".m",
    ".conf",
    ".overlay",
    ".yaml",
    ".yml",
    ".defconfig",
    ".sysbuild",
}
SPECIAL_FILENAMES = {"CMakeLists.txt", "Kconfig"}

LICENSE_TEXT = """Copyright 2026 Filip Dymczyk and Konrad Grucel"""


def get_comment_style(file: Path):
    if file.suffix in {".c", ".cpp", ".h", ".hpp", ".overlay"}:
        return "cpp"
    elif file.suffix == ".m":
        return "matlab"
    elif file.suffix in {".py", ".conf", ".yaml", ".yml", ".defconfig", ".sysbuild"}:
        return "py"
    elif file.name in SPECIAL_FILENAMES:
        return "hash"
    return None


def format_license(style: str):
    lines = LICENSE_TEXT.strip().splitlines()
    comment_style = ""
    if style == "cpp":
        comment_style = "//"
    elif style == "matlab":
        comment_style = "%"
    elif style in {"py", "hash"}:
        comment_style = "#"
    return "\n".join(f"{comment_style} {line}" for line in lines) + "\n\n"


def already_has_license(content: str):
    return "Copyright 2026 Filip Dymczyk and Konrad Grucel" in content


def process_file(file: Path):

    style = get_comment_style(file)
    if not style:
        return

    try:
        content = file.read_text(encoding="utf-8")
    except Exception as e:
        print(f"[ERROR] Could not read {file}: {e}")
        return

    if already_has_license(content):
        return

    license_block = format_license(style)

    if not license_block:
        return

    # Handle shebang for Python files
    if style == "py" and content.startswith("#!"):
        lines = content.splitlines()
        new_content = lines[0] + "\n\n" + license_block + "\n".join(lines[1:])
    else:
        new_content = license_block + content

    try:
        file.write_text(new_content, encoding="utf-8")
        print(f"[OK] Updated: {file}")
    except Exception as e:
        print(f"[ERROR] Could not write {file}: {e}")


def run(root: Path):

    if not root.exists():
        print(f"[ERROR] Directory does not exist: {root}")
        return

    count_total = 0
    count_candidates = 0

    for file in root.rglob("*"):
        count_total += 1

        if any(part in EXCLUDE_DIRS for part in file.parts):
            continue

        if file.is_file():
            if file.suffix in EXTENSIONS or file.name in SPECIAL_FILENAMES:
                count_candidates += 1
                process_file(file)

    print(f"\nScan summary:")
    print(f"  Total visited: {count_total}")
    print(f"  Candidate files: {count_candidates}")


if __name__ == "__main__":
    source_dir = Path("Sources")
    run(source_dir)
