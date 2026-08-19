FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
    
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

WORKDIR /app

# Install reforge-core from APT repository instead of requirements.txt
RUN apt-get update && apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://reforge-robotics.github.io/reforge-core-cpp/setup.sh | bash

RUN apt-get update && apt-get install -y \
    reforge-core \
    && rm -rf /var/lib/apt/lists/*

COPY src/robot/requirements.txt src/robot/requirements.txt
RUN pip install --no-cache-dir -r src/robot/requirements.txt
