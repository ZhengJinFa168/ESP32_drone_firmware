Import("env")
# Pre-build script: pin loop task to core 1, leaving core 0 for WiFi/BT
env.Append(BUILD_FLAGS=["-DARDUINO_RUNNING_CORE=1"])
print("setup_cores: loop pinned to core 1")
