import socket
import os
import platform

Import("env")

def get_local_ip():
    """Gets the local, outwardly facing, IP address of the computer"""
    try:
        # Create a socket (doesn't actually connect)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))  # Connect to a known external server
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception:
        return "127.0.0.1"  # Return localhost if we can't get the IP

# Get the computer's IP address
computer_ip = get_local_ip()

# Construct the upload command directly, ensuring separate arguments.
# This method is more reliable than appending to UPLOAD_FLAGS.
upload_command = [
    "$PYTHONEXE",
    "$PROJECT_DIR/.pio/libdeps/$PIOENV/ArduinoOTA/tools/espota.py", # Full path for windows
    "-i", "$UPLOAD_PORT",
    "-I", computer_ip,
    "--port", "3232",
     "-f", "$BUILD_DIR/${PROGNAME}.bin"
]

# Replace the default upload command with our custom command.
env.Replace(UPLOADCMD=" ".join(upload_command))

print(f"Computer IP Address: {computer_ip}")
print(f"Set UPLOADCMD to: {' '.join(upload_command)}")