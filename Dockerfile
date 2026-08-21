FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-dev \
    python3-venv \
    python3-pip \
    curl \
    cmake \
    ninja-build \    
    build-essential \
    && rm -rf /var/lib/apt/lists/*
    
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

WORKDIR /app
ENV PYTHONPATH="/app/src:$PYTHONPATH"

# reforge-core native Python build dependencies
RUN python -m pip install --upgrade pip setuptools wheel
RUN pip install --no-cache-dir \
    scikit-build-core \
    cmeel-eigen==3.4.1 \
    cmeel-urdfdom-headers==3.0.0 \
    pybind11 \
    nlohmann-json==3.12.0 \
    pin==4.0.0

# Rust 1.84.1 required by reforge-core 2.0.14
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --default-toolchain 1.84.1

ENV PATH="/root/.cargo/bin:$PATH"
ENV RUSTUP_TOOLCHAIN=1.84.1

# Build reforge-core 2.0.14 from its source distribution
RUN RUSTUP_TOOLCHAIN=1.84.1 \
    python -m pip install \
    --no-cache-dir \
    --force-reinstall \
    --no-build-isolation \
    --no-deps \
    reforge-core==2.0.14

# install requirements.txt
COPY src/robot/requirements.txt src/robot/requirements.txt
RUN pip install --no-cache-dir -r src/robot/requirements.txt

# pytest, for pyproject.toml's [project.optional-dependencies].dev.
# Installed by name rather than `pip install -e ".[dev]"`, which would
# re-resolve pyproject.toml's base `dependencies` (including unpinned
# reforge-core[all]) and risk clobbering the from-source reforge-core
# install above.
RUN pip install --no-cache-dir pytest

# # validate reforge_core install
# RUN python -c "\
# from reforge_core.control import _native_shaper; \
# print('complete_backend_available:', _native_shaper.complete_backend_available); \
# print('has NativeShaper:', hasattr(_native_shaper, 'NativeShaper')); \
# "

