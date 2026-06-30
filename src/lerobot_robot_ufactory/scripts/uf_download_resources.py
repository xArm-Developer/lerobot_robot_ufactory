#!/usr/bin/env python
"""Download XVSDK binaries and UMI shared libraries after install."""
import os
import sys
import urllib.request
import subprocess
import site

RESOURCES = "https://raw.githubusercontent.com/xArm-Developer/ufactory_resources/main/fastumi"

FILES = [
    # (url_path, subdir)
    ("libxvlib.so", ""),
    ("opencv/libopencv_core.so.4.2", ""),
    ("opencv/libopencv_imgproc.so.4.2", ""),
]

DEB_FILE = "sdk/XVSDK_focal_amd64.deb"


def get_pkg_dir():
    """Find the installed ufactory_lerobot package directory."""
    for sp in site.getsitepackages():
        path = os.path.join(sp, "ufactory_lerobot", "devices", "umi", "xvlib")
        if os.path.isdir(path):
            return path
    # fallback: try user site
    usp = site.getusersitepackages()
    if usp:
        return os.path.join(usp, "ufactory_lerobot", "devices", "umi", "xvlib")
    sys.exit("Error: cannot find installed ufactory_lerobot package.")


def download(url, dest):
    print(f"  -> {dest}")
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    urllib.request.urlretrieve(url, dest)


def main():
    pkg_dir = get_pkg_dir()
    print(f"Installing to: {pkg_dir}\n")

    for name, subdir in FILES:
        url = f"{RESOURCES}/{name}"
        dest = os.path.join(pkg_dir, os.path.basename(name))
        print(f"Downloading {name} ...")
        download(url, dest)

    # .deb: download to /tmp and install
    deb_url = f"{RESOURCES}/{DEB_FILE}"
    deb_path = "/tmp/XVSDK_focal_amd64.deb"
    print(f"\nDownloading {DEB_FILE} ...")
    download(deb_url, deb_path)

    print("\nInstalling XVSDK ...")
    ret = subprocess.run(["sudo", "dpkg", "-i", deb_path]).returncode
    if ret != 0:
        subprocess.run(["sudo", "apt", "install", "-y", "--fix-broken"])
    os.unlink(deb_path)

    print("\nDone.")


if __name__ == "__main__":
    main()