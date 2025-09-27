# Start from Ubuntu 24.04
FROM ubuntu:24.04

# Set environment variables for non-interactive installs
ENV DEBIAN_FRONTEND=noninteractive

# --- 1. INSTALL ALL SYSTEM DEPENDENCIES (as root) ---
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    git \
    gnupg \
    gstreamer1.0-gl \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    libfuse2 \
    libpulse-dev \
    lsb-release \
    sudo \
    wget \
    fuse \
    python3-pip \
    python3-venv \
    && apt-get remove -y modemmanager \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# --- 2. CREATE A NON-ROOT USER (as root) ---
RUN useradd --create-home --shell /bin/bash --user-group devuser && \
    usermod -aG sudo,dialout devuser && \
    echo "devuser ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# --- 3. SWITCH TO THE NEW USER ---
USER devuser
WORKDIR /home/devuser

# --- SETUP VIRTUAL ENVIRONMENT ---
# Create a virtual environment
RUN python3 -m venv /home/devuser/venv
# Add the venv's bin directory to the PATH
ENV PATH="/home/devuser/venv/bin:$PATH"

# --- 4. PREPARE THE APPIMAGE ---
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
RUN chmod +x ./QGroundControl-x86_64.AppImage

# --- 5. CLONE, SETUP, AND CLEAN THE PX4 TOOLCHAIN ---
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git && \
    /bin/bash -c "export USER=devuser && export HOME=/home/devuser && bash ./PX4-Autopilot/Tools/setup/ubuntu.sh"

# --- 6. Download a missing CMake file for building gazebo.
WORKDIR /home/devuser/PX4-Autopilot/Tools/simulation/gz
RUN wget https://raw.githubusercontent.com/gazebosim/gz-harmonic/refs/heads/main/CMakeLists.txt

# --- 7. Build the PX4 files for later use
WORKDIR /home/devuser/PX4-Autopilot
RUN make px4_sitl

# --- 8. INSTALL PYTHON LIBRARIES ---
# Install packages into the virtual environment. No special flags are needed
# because the PATH is already pointing to the venv's pip.
RUN pip install --no-cache-dir mavsdk pymavlink

# --- 9. Clone AZA repo ---
WORKDIR /home/devuser/PX4-Autopilot
RUN git clone https://github.com/ColeHersh/Arizona-Autonomous-Vehicles-Club-VTOL.git

# --- 10. CONFIGURE THE CONTAINER'S DEFAULT STATE ---
WORKDIR /home/devuser/PX4-Autopilot
CMD ["/bin/bash"]
