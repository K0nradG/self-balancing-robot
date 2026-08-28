#!/usr/bin/env python3
"""Provision an SD card for a Raspberry Pi Zero 2 W camera access point.

The script downloads and verifies Raspberry Pi OS Lite 32-bit and MediaMTX,
flashes an SD card, and configures:
  * a WPA2 Wi-Fi access point with a static 192.168.4.1 address,
  * SSH,
  * a MediaMTX RTSP server using the Raspberry Pi camera.

Run this script as root on a Linux host. The target block device is erased.

last login and password:

username: robot
password: sbrobot123

addres: ssh robot@192.168.4.1

"""

from __future__ import annotations

import argparse
import getpass
import hashlib
import json
import lzma
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import zipfile
from html.parser import HTMLParser
from pathlib import Path
from typing import Any, BinaryIO, Iterable

RPI_IMAGES_INDEX = "https://downloads.raspberrypi.com/raspios_lite_armhf/images/"
MEDIAMTX_RELEASE_API = (
    "https://api.github.com/repos/bluenviron/mediamtx/releases/latest"
)
USER_AGENT = "robot-rpi-sd-provisioner/1.0"
COPY_CHUNK_SIZE = 4 * 1024 * 1024
MIN_DEVICE_SIZE = 3_000_000_000


class ProvisionError(RuntimeError):
    """Expected provisioning failure."""


class LinkParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.links: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag != "a":
            return
        for key, value in attrs:
            if key == "href" and value:
                self.links.append(value)


def info(message: str) -> None:
    print(f"[+] {message}", flush=True)


def warn(message: str) -> None:
    print(f"[!] {message}", file=sys.stderr, flush=True)


def run(
    command: list[str],
    *,
    capture_output: bool = False,
    check: bool = True,
    input_text: str | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        check=check,
        capture_output=capture_output,
        text=True,
        input=input_text,
    )


def require_host_tools() -> None:
    required = ("lsblk", "mount", "umount", "openssl")
    missing = [tool for tool in required if shutil.which(tool) is None]
    if missing:
        raise ProvisionError(
            "Brak wymaganych programów: "
            + ", ".join(missing)
            + ". Zainstaluj je i uruchom skrypt ponownie."
        )


def request(url: str) -> urllib.request.Request:
    return urllib.request.Request(url, headers={"User-Agent": USER_AGENT})


def fetch_text(url: str) -> str:
    with urllib.request.urlopen(request(url), timeout=60) as response:
        return response.read().decode("utf-8")


def fetch_json(url: str) -> dict[str, Any]:
    return json.loads(fetch_text(url))


def parse_links(html: str) -> list[str]:
    parser = LinkParser()
    parser.feed(html)
    return parser.links


def discover_latest_rpi_image() -> tuple[str, str]:
    info("Wyszukiwanie najnowszego Raspberry Pi OS Lite 32-bit")
    release_links = [
        link
        for link in parse_links(fetch_text(RPI_IMAGES_INDEX))
        if re.fullmatch(r"raspios_lite_armhf-\d{4}-\d{2}-\d{2}/", link)
    ]
    if not release_links:
        raise ProvisionError("Nie znaleziono katalogów obrazów Raspberry Pi OS.")

    release_url = urllib.parse.urljoin(RPI_IMAGES_INDEX, max(release_links))
    image_names = [
        urllib.parse.unquote(link)
        for link in parse_links(fetch_text(release_url))
        if re.fullmatch(r"[^/]+-armhf-lite\.img\.xz", urllib.parse.unquote(link))
    ]
    if not image_names:
        raise ProvisionError(f"Nie znaleziono obrazu Lite 32-bit w {release_url}")

    image_url = urllib.parse.urljoin(release_url, max(image_names))
    return image_url, image_url + ".sha256"


def download(url: str, destination: Path) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        info(f"Użycie pliku z pamięci podręcznej: {destination}")
        return destination

    temporary = destination.with_suffix(destination.suffix + ".part")
    info(f"Pobieranie {url}")
    try:
        with urllib.request.urlopen(request(url), timeout=120) as response:
            total = int(response.headers.get("Content-Length", "0"))
            downloaded = 0
            next_report = 0
            with temporary.open("wb") as output:
                while chunk := response.read(COPY_CHUNK_SIZE):
                    output.write(chunk)
                    downloaded += len(chunk)
                    if downloaded >= next_report:
                        if total:
                            percent = downloaded * 100 // total
                            print(
                                f"\r    {downloaded // 1048576} MiB / "
                                f"{total // 1048576} MiB ({percent}%)",
                                end="",
                                flush=True,
                            )
                        else:
                            print(
                                f"\r    {downloaded // 1048576} MiB",
                                end="",
                                flush=True,
                            )
                        next_report = downloaded + 16 * 1024 * 1024
            print()
        temporary.replace(destination)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
    return destination


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(COPY_CHUNK_SIZE):
            digest.update(chunk)
    return digest.hexdigest()


def verify_sha256(path: Path, expected: str) -> None:
    expected = expected.lower().removeprefix("sha256:")
    info(f"Weryfikacja SHA-256 pliku {path.name}")
    actual = sha256_file(path)
    if actual != expected:
        raise ProvisionError(
            f"Błędna suma SHA-256 pliku {path}:\n"
            f"oczekiwano: {expected}\n"
            f"otrzymano: {actual}"
        )


def get_image_archive(
    cache_dir: Path, image_url: str | None, image_sha256: str | None
) -> Path:
    if image_url is None:
        image_url, checksum_url = discover_latest_rpi_image()
    else:
        checksum_url = image_url + ".sha256"

    filename = Path(urllib.parse.urlparse(image_url).path).name
    if not filename:
        raise ProvisionError("Adres obrazu nie zawiera nazwy pliku.")
    archive = download(image_url, cache_dir / filename)

    if image_sha256:
        expected = image_sha256.split()[0]
    else:
        info(f"Pobieranie sumy kontrolnej {checksum_url}")
        expected = fetch_text(checksum_url).split()[0]
    if not re.fullmatch(r"[0-9a-fA-F]{64}", expected):
        raise ProvisionError("Serwer zwrócił nieprawidłową sumę SHA-256 obrazu.")
    verify_sha256(archive, expected)
    return archive


def get_mediamtx(cache_dir: Path) -> Path:
    info("Wyszukiwanie najnowszego MediaMTX dla ARMv7")
    release = fetch_json(MEDIAMTX_RELEASE_API)
    candidates = [
        asset
        for asset in release.get("assets", [])
        if asset.get("name", "").endswith("_linux_armv7.tar.gz")
    ]
    if len(candidates) != 1:
        raise ProvisionError(
            "Nie udało się jednoznacznie znaleźć wydania MediaMTX ARMv7."
        )

    asset = candidates[0]
    archive = download(asset["browser_download_url"], cache_dir / asset["name"])
    digest = asset.get("digest")
    if digest:
        verify_sha256(archive, digest)
    else:
        warn(
            "GitHub nie podał sumy SHA-256 MediaMTX; archiwum nie może zostać "
            "niezależnie zweryfikowane."
        )

    binary = cache_dir / f"mediamtx-{release.get('tag_name', 'latest')}-armv7"
    if binary.exists():
        return binary

    info("Rozpakowywanie MediaMTX")
    with tarfile.open(archive, "r:gz") as package:
        members = [
            member
            for member in package.getmembers()
            if member.isfile() and Path(member.name).name == "mediamtx"
        ]
        if len(members) != 1:
            raise ProvisionError("Archiwum MediaMTX nie zawiera jednego pliku.")
        source = package.extractfile(members[0])
        if source is None:
            raise ProvisionError("Nie można odczytać programu MediaMTX.")
        with binary.open("wb") as output:
            shutil.copyfileobj(source, output, COPY_CHUNK_SIZE)
    binary.chmod(0o755)
    return binary


def lsblk_tree() -> list[dict[str, Any]]:
    result = run(
        [
            "lsblk",
            "--json",
            "--bytes",
            "--paths",
            "--output",
            "NAME,PATH,TYPE,RM,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS",
        ],
        capture_output=True,
    )
    return json.loads(result.stdout).get("blockdevices", [])


def walk_nodes(nodes: Iterable[dict[str, Any]]) -> Iterable[dict[str, Any]]:
    for node in nodes:
        yield node
        yield from walk_nodes(node.get("children") or [])


def get_device_node(device: str) -> tuple[str, dict[str, Any]]:
    resolved = os.path.realpath(device)
    try:
        mode = os.stat(resolved).st_mode
    except FileNotFoundError as error:
        raise ProvisionError(f"Urządzenie nie istnieje: {device}") from error
    if not stat.S_ISBLK(mode):
        raise ProvisionError(f"To nie jest urządzenie blokowe: {device}")

    for node in walk_nodes(lsblk_tree()):
        if os.path.realpath(node.get("path", "")) == resolved:
            if node.get("type") != "disk":
                raise ProvisionError(f"Podaj całą kartę, nie partycję: {resolved}")
            return resolved, node
    raise ProvisionError(f"lsblk nie rozpoznał urządzenia {resolved}.")


def all_mountpoints(node: dict[str, Any]) -> list[str]:
    mountpoints: list[str] = []
    for child in walk_nodes([node]):
        mountpoints.extend(
            mountpoint for mountpoint in (child.get("mountpoints") or []) if mountpoint
        )
    return mountpoints


def validate_target(
    device: str, node: dict[str, Any], force_non_removable: bool
) -> None:
    mountpoints = all_mountpoints(node)
    if "/" in mountpoints:
        raise ProvisionError(
            f"Odmowa wymazania {device}: znajduje się na nim system hosta."
        )
    if int(node.get("size") or 0) < MIN_DEVICE_SIZE:
        raise ProvisionError("Karta ma mniej niż 3 GB i jest za mała.")
    if not bool(node.get("rm")) and not force_non_removable:
        raise ProvisionError(
            f"{device} nie jest oznaczone jako wymienne. Jeśli to na pewno "
            "czytnik SD, dodaj --force-non-removable."
        )


def human_size(size: int) -> str:
    return f"{size / (1024**3):.1f} GiB"


def confirm_erase(device: str, node: dict[str, Any], assume_yes: bool) -> None:
    model = (node.get("model") or "nieznany model").strip()
    print(
        "\nUWAGA: następujące urządzenie zostanie CAŁKOWICIE WYMAZANE:\n"
        f"  urządzenie: {device}\n"
        f"  model:      {model}\n"
        f"  rozmiar:    {human_size(int(node.get('size') or 0))}\n"
    )
    if assume_yes:
        return
    expected = f"WIPE {device}"
    answer = input(f"Aby kontynuować, wpisz dokładnie „{expected}”: ")
    if answer != expected:
        raise ProvisionError("Anulowano zapis karty.")


def device_identity(node: dict[str, Any]) -> tuple[int, str, str]:
    return (
        int(node.get("size") or 0),
        (node.get("model") or "").strip(),
        (node.get("serial") or "").strip(),
    )


def unmount_device(node: dict[str, Any]) -> None:
    mountpoints = sorted(all_mountpoints(node), key=len, reverse=True)
    for mountpoint in mountpoints:
        info(f"Odmontowywanie {mountpoint}")
        run(["umount", mountpoint])


def copy_with_progress(source: BinaryIO, destination: BinaryIO) -> None:
    written = 0
    next_report = 0
    while chunk := source.read(COPY_CHUNK_SIZE):
        destination.write(chunk)
        written += len(chunk)
        if written >= next_report:
            print(
                f"\r    zapisano {written // 1048576} MiB",
                end="",
                flush=True,
            )
            next_report = written + 64 * 1024 * 1024
    print()


def flash_image(archive: Path, device: str) -> None:
    info(f"Zapisywanie obrazu na {device}")
    with open(device, "wb", buffering=0) as destination:
        if archive.name.endswith(".xz"):
            with lzma.open(archive, "rb") as source:
                copy_with_progress(source, destination)
        elif archive.name.endswith(".zip"):
            with zipfile.ZipFile(archive) as package:
                images = [name for name in package.namelist() if name.endswith(".img")]
                if len(images) != 1:
                    raise ProvisionError(
                        "Archiwum ZIP nie zawiera dokładnie jednego obrazu."
                    )
                with package.open(images[0]) as source:
                    copy_with_progress(source, destination)
        elif archive.name.endswith(".img"):
            with archive.open("rb") as source:
                copy_with_progress(source, destination)
        else:
            raise ProvisionError(
                "Obsługiwane obrazy mają rozszerzenie .img.xz, .zip lub .img."
            )
        destination.flush()
        os.fsync(destination.fileno())
    os.sync()

    if shutil.which("partprobe"):
        run(["partprobe", device], check=False)
    elif shutil.which("blockdev"):
        run(["blockdev", "--rereadpt", device], check=False)
    if shutil.which("udevadm"):
        run(["udevadm", "settle"], check=False)


def find_partitions(device: str) -> tuple[str, str]:
    for _ in range(15):
        nodes = lsblk_tree()
        parent = next(
            (
                node
                for node in nodes
                if os.path.realpath(node.get("path", "")) == device
            ),
            None,
        )
        children = (parent or {}).get("children") or []
        boot = next(
            (
                child["path"]
                for child in children
                if child.get("fstype") in ("vfat", "fat", "fat32")
                or child.get("label") == "bootfs"
            ),
            None,
        )
        root = next(
            (
                child["path"]
                for child in children
                if child.get("fstype") == "ext4" or child.get("label") == "rootfs"
            ),
            None,
        )
        if boot and root:
            return boot, root
        time.sleep(1)
    raise ProvisionError("Nie znaleziono partycji bootfs i rootfs po zapisaniu obrazu.")


def password_hash(password: str) -> str:
    result = run(
        ["openssl", "passwd", "-6", "-stdin"],
        capture_output=True,
        input_text=password,
    )
    value = result.stdout.strip()
    if not value.startswith("$6$"):
        raise ProvisionError("OpenSSL nie wygenerował poprawnego hasha hasła.")
    return value


def write_text(path: Path, content: str, mode: int = 0o644) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    path.chmod(mode)


def append_cmdline_option(path: Path, option: str) -> None:
    content = path.read_text(encoding="utf-8").strip()
    key = option.split("=", 1)[0] + "="
    values = [value for value in content.split() if not value.startswith(key)]
    values.append(option)
    write_text(path, " ".join(values) + "\n")


def configure_boot(
    boot: Path,
    username: str,
    user_password_hash: str,
    country: str,
) -> None:
    (boot / "ssh").touch(mode=0o644)
    write_text(boot / "userconf.txt", f"{username}:{user_password_hash}\n")

    cmdline = boot / "cmdline.txt"
    if not cmdline.exists():
        raise ProvisionError("Na partycji bootfs brakuje cmdline.txt.")
    append_cmdline_option(cmdline, f"cfg80211.ieee80211_regdom={country}")
    append_cmdline_option(cmdline, "console=serial0,115200")

    config = boot / "config.txt"
    config_content = config.read_text(encoding="utf-8")

    additions = []
    if "camera_auto_detect=1" not in config_content:
        additions.append("camera_auto_detect=1")
    if "enable_uart=1" not in config_content:
        additions.append("enable_uart=1")

    if additions:
        with config.open("a", encoding="utf-8") as output:
            output.write(
                "\n# Robot camera provisioning\n" + "\n".join(additions) + "\n"
            )


def configure_hostname(root: Path, hostname: str) -> None:
    write_text(root / "etc/hostname", hostname + "\n")
    hosts_path = root / "etc/hosts"
    hosts = hosts_path.read_text(encoding="utf-8")
    if re.search(r"^127\.0\.1\.1\s+", hosts, flags=re.MULTILINE):
        hosts = re.sub(
            r"^127\.0\.1\.1\s+.*$",
            f"127.0.1.1\t{hostname}",
            hosts,
            flags=re.MULTILINE,
        )
    else:
        hosts += f"\n127.0.1.1\t{hostname}\n"
    write_text(hosts_path, hosts)


def configure_access_point(
    root: Path,
    ssid: str,
    ap_password: str,
    channel: int,
) -> None:
    profile = f"""[connection]
id=robot-ap
uuid={uuid.uuid4()}
type=wifi
interface-name=wlan0
autoconnect=true

[wifi]
band=bg
channel={channel}
mode=ap
ssid={ssid}

[wifi-security]
key-mgmt=wpa-psk
psk={ap_password}

[ipv4]
address1=192.168.4.1/24
method=shared

[ipv6]
method=disabled
"""
    write_text(
        root / "etc/NetworkManager/system-connections/robot-ap.nmconnection",
        profile,
        mode=0o600,
    )


def configure_mediamtx(
    root: Path,
    mediamtx_binary: Path,
    width: int,
    height: int,
    fps: int,
    bitrate: int,
) -> None:
    destination = root / "usr/local/bin/mediamtx"
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(mediamtx_binary, destination)
    destination.chmod(0o755)

    configuration = f"""logLevel: info
rtsp: yes
rtspAddress: :8554
rtspTransports: [tcp]

paths:
  camera:
    source: rpiCamera
    rpiCameraWidth: {width}
    rpiCameraHeight: {height}
    rpiCameraFPS: {fps}
    rpiCameraBitrate: {bitrate}
"""
    write_text(root / "etc/mediamtx.yml", configuration)

    service = """[Unit]
Description=Robot camera RTSP server
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
ExecStart=/usr/local/bin/mediamtx /etc/mediamtx.yml
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
"""
    unit = root / "etc/systemd/system/robot-camera.service"
    write_text(unit, service)
    wants = root / "etc/systemd/system/multi-user.target.wants"
    wants.mkdir(parents=True, exist_ok=True)
    link = wants / "robot-camera.service"
    link.unlink(missing_ok=True)
    link.symlink_to("../robot-camera.service")


def provision_filesystems(
    device: str,
    username: str,
    user_password: str,
    hostname: str,
    country: str,
    ssid: str,
    ap_password: str,
    channel: int,
    mediamtx_binary: Path,
    width: int,
    height: int,
    fps: int,
    bitrate: int,
) -> None:
    boot_partition, root_partition = find_partitions(device)
    # Desktop automounters can mount fresh partitions immediately after flashing.
    _, refreshed_node = get_device_node(device)
    unmount_device(refreshed_node)

    with tempfile.TemporaryDirectory(prefix="robot-rpi-") as temporary:
        temporary_path = Path(temporary)
        boot_mount = temporary_path / "boot"
        root_mount = temporary_path / "root"
        boot_mount.mkdir()
        root_mount.mkdir()

        info(f"Montowanie {root_partition}")
        run(["mount", root_partition, str(root_mount)])
        root_mounted = True
        boot_mounted = False
        try:
            info(f"Montowanie {boot_partition}")
            run(["mount", boot_partition, str(boot_mount)])
            boot_mounted = True

            configure_boot(
                boot_mount,
                username,
                password_hash(user_password),
                country,
            )
            configure_hostname(root_mount, hostname)
            configure_access_point(root_mount, ssid, ap_password, channel)
            configure_mediamtx(
                root_mount,
                mediamtx_binary,
                width,
                height,
                fps,
                bitrate,
            )
            os.sync()
        finally:
            if boot_mounted:
                run(["umount", str(boot_mount)], check=False)
            if root_mounted:
                run(["umount", str(root_mount)], check=False)


def valid_hostname(value: str) -> str:
    if not re.fullmatch(r"[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?", value):
        raise argparse.ArgumentTypeError("Nieprawidłowa nazwa hosta.")
    return value.lower()


def valid_username(value: str) -> str:
    if not re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", value):
        raise argparse.ArgumentTypeError("Nieprawidłowa nazwa użytkownika.")
    return value


def valid_country(value: str) -> str:
    if not re.fullmatch(r"[A-Za-z]{2}", value):
        raise argparse.ArgumentTypeError("Kod kraju musi mieć dwie litery, np. PL.")
    return value.upper()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Zapisuje Raspberry Pi OS Lite 32-bit na kartę SD i konfiguruje "
            "Pi Zero 2 W jako AP z serwerem kamery RTSP."
        )
    )
    parser.add_argument(
        "--device",
        required=True,
        help="Całe urządzenie karty SD, np. /dev/sdb (NIE /dev/sdb1).",
    )
    parser.add_argument("--username", type=valid_username, default="robot")
    parser.add_argument("--hostname", type=valid_hostname, default="robotcam")
    parser.add_argument("--country", type=valid_country, default="PL")
    parser.add_argument("--ssid", default="RobotCam")
    parser.add_argument(
        "--ap-password",
        help="Hasło WPA2; bez tej opcji skrypt zapyta o nie bez wyświetlania.",
    )
    parser.add_argument(
        "--user-password",
        help="Hasło SSH; bez tej opcji skrypt zapyta o nie bez wyświetlania.",
    )
    parser.add_argument(
        "--channel",
        type=int,
        choices=(1, 6, 11),
        default=6,
        help="Kanał Wi-Fi 2,4 GHz (domyślnie 6).",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="Bitrate H.264 w bit/s.",
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        default=Path.home() / ".cache/robot-rpi-provision",
    )
    parser.add_argument(
        "--image-url",
        help="Opcjonalny własny adres obrazu .img.xz zamiast najnowszego.",
    )
    parser.add_argument(
        "--image-sha256",
        help="SHA-256 własnego obrazu; domyślnie pobierany plik .sha256.",
    )
    parser.add_argument(
        "--force-non-removable",
        action="store_true",
        help="Zezwól na urządzenie nieoznaczone przez system jako wymienne.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Pomiń końcowe potwierdzenie wymazania (niezalecane).",
    )
    return parser.parse_args()


def get_secret(supplied: str | None, prompt: str, minimum_length: int) -> str:
    value = supplied if supplied is not None else getpass.getpass(prompt)
    if len(value) < minimum_length:
        raise ProvisionError(f"Hasło musi mieć co najmniej {minimum_length} znaków.")
    if "\n" in value or "\r" in value or "\x00" in value:
        raise ProvisionError("Hasło zawiera niedozwolony znak.")
    return value


def main() -> int:
    args = parse_arguments()
    try:
        if sys.platform != "linux":
            raise ProvisionError("Ten skrypt działa wyłącznie na Linuksie.")
        if os.geteuid() != 0:
            raise ProvisionError("Uruchom skrypt jako root, np. przez sudo.")
        require_host_tools()

        if not (1 <= len(args.ssid.encode("utf-8")) <= 32):
            raise ProvisionError("SSID musi mieć od 1 do 32 bajtów.")
        if args.width <= 0 or args.height <= 0:
            raise ProvisionError("Wymiary obrazu muszą być dodatnie.")
        if not 1 <= args.fps <= 60:
            raise ProvisionError("FPS musi należeć do zakresu 1–60.")
        if args.bitrate < 100_000:
            raise ProvisionError("Bitrate musi wynosić co najmniej 100000.")

        ap_password = get_secret(args.ap_password, "Hasło sieci RobotCam: ", 8)
        if len(ap_password.encode("utf-8")) > 63:
            raise ProvisionError("Hasło WPA2 może mieć maksymalnie 63 bajty.")
        user_password = get_secret(args.user_password, "Hasło użytkownika SSH: ", 8)

        device, node = get_device_node(args.device)
        validate_target(device, node, args.force_non_removable)
        confirm_erase(device, node, args.yes)
        confirmed_identity = device_identity(node)

        image = get_image_archive(args.cache_dir, args.image_url, args.image_sha256)
        mediamtx = get_mediamtx(args.cache_dir)

        # Check the device again after downloads, in case it was unplugged or
        # device names changed while the potentially long downloads ran.
        device, node = get_device_node(device)
        validate_target(device, node, args.force_non_removable)
        if device_identity(node) != confirmed_identity:
            raise ProvisionError(
                "Urządzenie pod wskazaną nazwą zmieniło się podczas "
                "pobierania. Uruchom skrypt ponownie i potwierdź właściwą kartę."
            )
        unmount_device(node)
        flash_image(image, device)
        provision_filesystems(
            device=device,
            username=args.username,
            user_password=user_password,
            hostname=args.hostname,
            country=args.country,
            ssid=args.ssid,
            ap_password=ap_password,
            channel=args.channel,
            mediamtx_binary=mediamtx,
            width=args.width,
            height=args.height,
            fps=args.fps,
            bitrate=args.bitrate,
        )

        info("Karta SD została przygotowana.")
        print(
            "\nPo uruchomieniu Raspberry Pi:\n"
            f"  Wi-Fi:       {args.ssid}\n"
            "  adres RPi:   192.168.4.1\n"
            "  RTSP:        rtsp://192.168.4.1:8554/camera\n"
            f"  SSH:         ssh {args.username}@192.168.4.1\n"
            "\nMożesz teraz bezpiecznie wyjąć kartę SD."
        )
        return 0
    except (
        ProvisionError,
        OSError,
        subprocess.CalledProcessError,
        urllib.error.URLError,
    ) as error:
        print(f"\nBŁĄD: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
