import platform
import os

print("Hello from the Raspberry Pi! - edited on Mac!  ")
print("Machine:", platform.machine())      # should say aarch64 or armv7l
print("System:", platform.system())
print("Release:", platform.release())
print("Current directory:", os.getcwd())