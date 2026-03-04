"""Clean all user files from the Pico's filesystem.

Usage: mpremote run clean_pico.py

Removes everything except boot.py, then reboots.
"""
import os


def rm_rf(path):
    """Recursively remove a file or directory."""
    try:
        for entry in os.listdir(path):
            rm_rf(path + "/" + entry)
        os.rmdir(path)
    except OSError:
        os.remove(path)


for name in os.listdir("/"):
    if name != "boot.py":
        print("Removing:", name)
        rm_rf("/" + name)

print("Pico filesystem is clean. Rebooting...")
import machine
machine.reset()
