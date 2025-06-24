# Polarization Sensitive Optical Coherence Tomography Angiography (PS-OCTA) and Optical Coherence Elastography (OCE) Robot Platform. 

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/index.html)
[![LabVIEW 2024 Q1](https://img.shields.io/badge/LabVIEW-2024%20Q1-yellow.svg)](https://www.ni.com/en/shop/labview.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

| Component | Tested Version | Notes |
|-----------|---------------|-------|
| ROS 2 | **Jazzy** | Native install or inside the provided Docker image |
| LabVIEW | **2024 Q1** (64-bit) | Required for acquisition & real‑time display |
| RTI DDS Toolkit | **3.1** | Install into LabVIEW before first run |
| Open3D | ≥ 0.18 | libopen3d-dev (debian) or from source |
| OpenCV | ≥ 4.10 | libopencv-dev |
| Eigen | ≥ 3.4 | Included with ROS2 by default or libeigen3-dev (debian) or from source  |
| Docker Desktop | ≥ 4.30 (optional) | Reproducible container build |

---

## Table of Contents
1. [Quick start](#quick-start)  
2. [Design](#repository)  
3. [Usage examples](#usage-examples)  
4. [Hardware](#hardware)
5. [Citing](#citing) 
6. [Funding](#funding)  

---

## Quick start
### Prerequisites
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- [LabVIEW 2024 Q1](https://www.ni.com/en/shop/labview.html)
- [RTI DDS Toolkit for LabVIEW](https://www.rti.com/products/tools/dds-toolkit-labview)
- [Open3D](https://www.open3d.org)
- [OpenCV](https://www.open3d.org)
- [Eigen](https://eigen.tuxfamily.org)
- [Docker (optional)](https://www.docker.com)

> All dependencies—except LabVIEW—are already baked into the Docker image.

### Docker Image (recommended)
```bash
cd docker/
docker compose up -d
```

### Native build
```bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Launch helper
```bash
./launch.sh -h
Launch octa/oce ROS program

Syntax: [-s|-d|-h]
options:
h     Print this Help.
s     Simulation
d     Debug
```

## Design [TODO]

## Usage examples [TODO]

## Citing [TODO]
```bibtex
@software{ps-octa-oce-robot,
  title        = {Polarization-Sensitive OCT Angiography and Optical Coherence Elastography Robot Platform},
  year         = {2025},
  publisher    = {GitHub},
  journal      = {GitHub repository},
}
```
  <!-- howpublished = {\url{https://github.com/<YOUR_ORG>/ps-octa-oce-robot}}, -->
  <!-- doi          = {10.5281/zenodo.1234567} -->

## Hardware [TODO]
- UR3e Robot arm

## Funding

This work was supported by the National Institutes of Health under grant R01AR077560-01A1 – “Dynamic OCE with acoustic micro-tapping for in‑vivo monitoring of skin-graft surgeries”, University of Washington, PI Ivan Pelivanov.
