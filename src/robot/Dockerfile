# Public package image template. The exporter renders the integration token
# before publishing a target-specific package tree.
FROM python:3.11-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    PYTHONUNBUFFERED=1

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      git \
      libeigen3-dev \
      libfftw3-dev \
      pkg-config \
      wget \
      unzip \
      zlib1g-dev && \
    rm -rf /var/lib/apt/lists/*

ARG ROBOT_INTEGRATION_ID=trossen_arm
ARG SDK_CANDIDATE_URL=https://files.pythonhosted.org/packages/b9/11/4b4061f0dad7d20951e843d390b7409d4e335c259efe34de8a2f036e1152/reforge_core-2.0.22-cp311-cp311-manylinux_2_28_x86_64.whl
ARG SDK_CANDIDATE_SHA256=8beda2097e728a7e3f11c7fd496d5534cc70494737bc6b408256e2c57973d0b2
ARG SDK_CANDIDATE_FILENAME=reforge_core-2.0.22-cp311-cp311-manylinux_2_28_x86_64.whl
ENV REFORGE_ROBOT_DATA_DIR=/opt/reforge-interface/src/robot/data \
    REFORGE_ROBOT_MODELS_DIR=/opt/reforge-interface/src/robot/models

WORKDIR /opt/reforge-interface
ENV VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:$PATH \
    PYTHONPATH=/opt/reforge-interface

RUN python -m venv "$VIRTUAL_ENV" && \
    mkdir -p "$REFORGE_ROBOT_DATA_DIR" "$REFORGE_ROBOT_MODELS_DIR"

COPY requirements.txt pyproject.toml README.md ./
RUN set -eu; \
    phase5_sentinel='PHASE5_'; \
    phase5_sentinel="${phase5_sentinel}REQUIRED"; \
    if [ "$SDK_CANDIDATE_URL" = "$phase5_sentinel" ] || \
       [ "$SDK_CANDIDATE_SHA256" = "$phase5_sentinel" ] || \
       [ "$SDK_CANDIDATE_FILENAME" = "$phase5_sentinel" ]; then \
      echo "Phase 5 SDK candidate URL, SHA256, and filename must be reviewed before this image can be built." >&2; \
      exit 1; \
    fi; \
    case "$SDK_CANDIDATE_URL" in \
      https://?*) ;; \
      *) echo "candidate URL must use HTTPS" >&2; exit 1 ;; \
    esac; \
    if ! printf '%s\n' "$SDK_CANDIDATE_SHA256" | grep -Eq '^[0-9a-fA-F]{64}$'; then \
      echo "candidate SHA256 must be exactly 64 hexadecimal characters" >&2; \
      exit 1; \
    fi; \
    case "$SDK_CANDIDATE_FILENAME" in \
      [A-Za-z0-9]*.whl) ;; \
      *) echo "candidate filename must be a conservative .whl basename" >&2; exit 1 ;; \
    esac; \
    case "$SDK_CANDIDATE_FILENAME" in \
      *..*|*[!A-Za-z0-9._-]*) \
        echo "candidate filename must not contain path traversal or unsafe characters" >&2; \
        exit 1; \
        ;; \
    esac; \
    candidate_path="/tmp/${SDK_CANDIDATE_FILENAME}"; \
    wget --https-only --no-verbose --output-document="$candidate_path" "$SDK_CANDIDATE_URL"; \
    printf '%s  %s\n' "$SDK_CANDIDATE_SHA256" "$candidate_path" | sha256sum --check --status; \
    python -c 'import email,re,sys,zipfile; wheel=zipfile.ZipFile(sys.argv[1]); paths=[name for name in wheel.namelist() if name.endswith(".dist-info/METADATA")]; assert len(paths)==1; metadata=email.message_from_bytes(wheel.read(paths[0])); assert re.sub(r"[-_.]+","-",metadata["Name"]).lower()=="reforge-core"; candidate_version=metadata["Version"]; assert candidate_version and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.!+_-]*",candidate_version); print(candidate_version)' \
      "$candidate_path" > /opt/reforge-core-candidate-version; \
    python -m pip install --no-cache-dir --no-deps "$candidate_path"; \
    rm -f "$candidate_path"
RUN python -m pip install --no-cache-dir -r requirements.txt

COPY . .
RUN python -m pip install --no-cache-dir -e . && \
    python -m pip check && \
    python -c 'from importlib.metadata import version; from pathlib import Path; assert version("reforge-core")==Path("/opt/reforge-core-candidate-version").read_text().strip()'
