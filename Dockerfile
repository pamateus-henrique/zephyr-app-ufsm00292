FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
    wget python3-pip python3-setuptools python3-wheel xz-utils file \
    build-essential libusb-1.0-0-dev libudev-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install edbg
RUN git clone https://github.com/ataradov/edbg.git /opt/edbg \
    && make -C /opt/edbg \
    && cp /opt/edbg/edbg /usr/local/bin/edbg \
    && rm -rf /opt/edbg

# Install west
RUN pip3 install --upgrade west

# Set environment variables
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/toolchains/zephyr-sdk
ENV ZEPHYR_BASE=/zephyr

# Download and install Zephyr SDK
RUN mkdir -p /opt/toolchains \
    && wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.4/zephyr-sdk-0.16.4_linux-x86_64.tar.xz \
    && tar -xf zephyr-sdk-0.16.4_linux-x86_64.tar.xz -C /opt/toolchains \
    && rm zephyr-sdk-0.16.4_linux-x86_64.tar.xz \
    && /opt/toolchains/zephyr-sdk-0.16.4/setup.sh -t all -h -c

WORKDIR /workspace

# Copy ONLY west.yml to leverage Docker layer caching
COPY west.yml .

# Initialize west and download Zephyr
RUN west init -l . && west update

# Install Zephyr's Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install --use-deprecated=legacy-resolver -r /zephyr/scripts/requirements.txt

# Don't build here! We'll build in the container with mounted code.
CMD ["/bin/bash"]