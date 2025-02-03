# Osprey Simulator

**Osprey Simulator** is an integrated simulation environment built upon my fork of the Pegasus Simulator. This repository brings together three major components:

- **Fixed wing simulation**: A simulator for fixed-wing UAV simulations.
- **Soar_Detect**: A deep learning-based wind vector detection and inference module.
- **Synthetic Data Generation**: A module (from the original Pegasus Simulator fork) used for generating synthetic simulation data and running fixed-wing dynamics simulations.
- **SoarEnvGen**: A CFD/OpenFOAM pipeline for environment generation.

Each component has its own README with detailed instructions. Please refer to the respective subfolders for more information on installation, usage, and configuration.

> **Note:** The folder `extensions/pegasus.simulator/pegasus/simulator/assets` is intentionally left empty because asset files could not be uploaded to GitHub. To generate these files, please use SoarEnvGen or contact me directly.

For further context, the simulation part of this project is based on the [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator).

---

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Installation and Setup](#installation-and-setup)
  - [Required Software](#required-software)
  - [Installation Instructions](#installation-instructions)
- [Usage](#usage)
  - [Soar_Detect](#soar_detect)
  - [Synthetic Data Generation](#synthetic-data-generation)
  - [SoarEnvGen](#soarenvgen)
- [Assets Note](#assets-note)
- [License](#license)

---

## Overview

**Osprey Simulator** combines the power of a well-established simulation engine (Pegasus Simulator) with advanced modules for synthetic data generation, CFD environment generation, and wind detection using deep learning. This unified repository enables streamlined simulation workflows, model training, and inference for fixed-wing dynamics and wind vector estimation.

The high-level architecture of the simulation environment is illustrated below:



The system integrates NVIDIA Isaac Sim for physics-based simulation, synthetic data generation, and sensor modeling. The Pegasus Interface manages vehicle operations, and assets such as fixed-wing vehicles, wind field data, and environments are stored in .usd and .npy formats.

---

## Project Structure

```
OspreySimulator/
├── extensions/
│   └── pegasus.simulator/
│       └── pegasus/
│           └── simulator/
│               |── assets/  # Asset files are not included – generate with SoarEnvGen or request them.
|               ├── logic/
|               │   ├── atmosphere.py
|               │   ├── dynamics.py
|               │   ├── sensors.py
|               │   └── vehicle.py
|               ├── state.py  # Manages simulation states
|               └── vehicle_manager.py  # Handles vehicle interactions
├── soar_detect/  # Contains the wind detection module (see its README for details)
├── synthetic_data_generation/  # Contains the Pegasus Simulator fork code & synthetic data generation scripts
└── soarenv/  # Contains the CFD/OpenFOAM pipeline (SoarEnvGen) for environment generation
```

- **soar_detect/**:  
  Refer to [soar_detect/README.md](soar_detect/README.md) for instructions on data preparation, training, evaluation, and inference using the U-Net-based model for wind vector detection.

- **synthetic_data_generation/**:  
  This folder contains my Pegasus Simulator fork, where I run the simulations and fixed-wing dynamics. Check the README in this folder for detailed simulation and data generation instructions.

- **soarenv/**:  
  Contains the SoarEnvGen pipeline that generates CFD/OpenFOAM simulation environments. For detailed setup and usage instructions, see [soarenv/README.md](soarenv/README.md).

---
### Simulator Folder

The `simulator/` folder contains core logic components that drive the simulation framework:

- logic/: This subfolder includes essential models:
  - atmosphere.py: Simulates atmospheric conditions, including wind and air density variations.
  - dynamics.py: Implements fixed-wing aircraft physics and aerodynamic forces.
  - sensors.py: Simulates onboard sensors for environment perception.
  - vehicle.py: Defines vehicle properties and interactions with the physics engine.

- state.py: Manages the overall state of the simulation, keeping track of the environment and vehicles.

- vehicle_manager.py: Oversees vehicle control, initialization, and interaction with the simulation engine.

These modules collectively ensure that Osprey Simulator provides a realistic simulation of fixed-wing flight dynamics and sensor-based perception.

## Installation and Setup

### Required Software

The following software has been tested on **Ubuntu 20.04 LTS**:

- **Omniverse Launcher & Isaac Sim**  
  - [Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/launcher/)  
  - **Isaac Sim** (Install with the Omniverse Launcher; tested with version **2022.2.0**)
- **Blender**
- **OpenFOAM** (v2212)
- **ROS (Noetic)**

### Installation Instructions

#### Omniverse Launcher & Isaac Sim

1. **Download, install, and run the Omniverse Launcher.**  
   It is strongly recommended to install it with the cache enabled. For troubleshooting, refer to the Omniverse documentation.

2. **Create a Local Nucleus Server:**  
   Go to the 'Nucleus' tab in the Launcher and enable it. You will need to create an associated local Nucleus account (this is not linked to any other account).

3. **Install Isaac Sim:**  
   - Go to the 'Exchange' tab in the Launcher and select Isaac Sim.
   - Note: The default installation location is `~/.local/share/ov/pkg/isaac_sim-<version>`.

4. **Download the Necessary Content:**  
   - From the 'Exchange' tab, download both the **Industrial 3D Models Pack** and the **USD Physics Sample Pack** by clicking 'Save as...'.
   - From the 'Nucleus' tab, navigate to `localhost/NVIDIA/Assets/Isaac/<version>/Isaac/Environments/Simple_Warehouse` and download the contents via the download button.
   - Extract these folders to `~/Omniverse_content`. (You can specify another location in AutoGDM2 settings if needed.)

---

## Usage

### Soar_Detect
- **Purpose:** Wind vector detection and inference using deep learning.
- **Instructions:** See `soar_detect/README.md` for guidance on data preparation, training, evaluation, and inference.

### Synthetic Data Generation
- **Purpose:** Run simulations and generate synthetic data using the Pegasus Simulator fork, including fixed-wing dynamics.
- **Instructions:** Refer to `synthetic_data_generation/README.md` for detailed simulation and data generation instructions.

### SoarEnvGen
- **Purpose:** Generate simulation environments using the CFD/OpenFOAM pipeline.
- **Instructions:** For installation and usage, see `soarenv/README.md`.

---

## Assets Note

The folder located at `extensions/pegasus.simulator/pegasus/simulator/assets` does not contain actual asset files due to GitHub file size restrictions.

To obtain these files:
- **Generate them:** Use SoarEnvGen as described in its README.
- **Request them:** Contact me directly if you require the asset files.

---

## License

This project is licensed under the MIT License (or insert your preferred license).

Feel free to contribute, open issues, or request additional features. For detailed instructions, consult the README files in each subfolder or refer to the Pegasus Simulator documentation.