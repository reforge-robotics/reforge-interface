#!/usr/bin/env bash
set -euo pipefail

# Install the matching public base and ROS 2 companion packages. The package
# version can be overridden when a later compatible release is approved.
# The helper configures Reforge's signed public APT source first; repository
# credentials and keys are intentionally not stored in this repository.
apt_setup_url="${REFORGE_APT_SETUP_URL:-https://reforge-robotics.github.io/reforge-core-cpp/setup.sh}"
package_version="${REFORGE_SHAPER_PACKAGE_VERSION:-2.0.15-1}"
curl -fsSL "${apt_setup_url}" | sudo bash
sudo apt-get update
sudo apt-get install -y \
  "reforge-core-shaper=${package_version}" \
  "reforge-core-shaper-ros2=${package_version}"
