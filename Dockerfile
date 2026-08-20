# UFACTORY does not require ROS 2 for direct xArm SDK control.
FROM python:3.11-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      git \
      libeigen3-dev \
      libfftw3-dev \
      pkg-config \
      unzip \
      wget \
      zlib1g-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /control-box-bot/reforge-interface

ENV VIRTUAL_ENV=/opt/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN python -m venv "$VIRTUAL_ENV"

COPY requirements.txt pyproject.toml MANIFEST.in README.md ./
RUN python -m pip install --no-cache-dir -r requirements.txt

COPY . .
RUN python -m pip install --no-cache-dir .

ENV PYTHONPATH=/control-box-bot/reforge-interface/src
ENV PYTHONUNBUFFERED=1

# Enable core dumps for debugging.
RUN echo 'ulimit -c unlimited' >> /etc/bash.bashrc
ENV SEGFAULT_SIGNALS="all"
