# Base image for tools/bsim/sim.sh (nrf52_bsim builds); "sim.sh setup" adds BabbleSim.
FROM --platform=linux/amd64 ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake ninja-build gperf ccache device-tree-compiler wget xz-utils file make \
      gcc g++ gcc-multilib g++-multilib python3 python3-pip python3-venv python3-dev \
      ca-certificates rsync \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /opt/venv && /opt/venv/bin/pip install --no-cache-dir west ninja
ENV PATH=/opt/venv/bin:$PATH \
    ZEPHYR_TOOLCHAIN_VARIANT=host \
    BSIM_OUT_PATH=/opt/bsim \
    BSIM_COMPONENTS_PATH=/opt/bsim/components
WORKDIR /work
