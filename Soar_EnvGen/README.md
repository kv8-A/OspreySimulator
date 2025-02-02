# SOAR_ENVGEN

This repository is used in conjunction with Osprey Simulator to generate soaring environments and their corresponding wind fields.

## Installation

## Blender Installation

Blender is required for environment generation.

1. Download and install Blender from the official website.

2. To use the environment generation scripts within Blender, set up a Conda environment:
```
conda create --name myblender_env python=3.10
conda activate myblender_env
pip install bpy
```
3. Blender Development Extension Setup in VS Code:
    - Install the Blender Development extension.
    - Press Ctrl+Shift+P → Select Start Blender → Choose the correct Blender version.
    - Press Ctrl+Shift+P → Select Run Script to execute scripts from VS Code.

4. (TODO: Create a Conda environment YAML for easy setup.)

## Functionality
### Sub-Functionalities

1. Generating Real-World Environments
    - Uses the Blender Add-on Blosm to generate real-world environments and exports them to USD format.

2. Generating Random Urban Environments
- Creates randomized, textured urban environments in Blender and exports them to USD format.(TODO: Insert command for running random_field.py.)

3. Automated OpenFOAM CFD Simulations
    - Converts Blender/ USD files to OBJ format for use in OpenFOAM.
    - Runs OpenFOAM simulations automatically and exports the wind field data.

4. Wind Field Generation
- Generates wind fields for selected environments at various wind velocities and directions.

### Main Functionalities

This repository primarily creates realistic wind fields for simulations in **[NVIDIA's Isaac SIM](https://developer.nvidia.com/isaac-sim)**, but it is compatible with other platforms.

#### Running Wind Field Generation

Use the following command with appropriate arguments:

```bash
python <filename.py> --windfield_folder --wind_velocities --wind_directions --random_env:bool --environment_folder:str default=None --n_of_random_envs:int
```

- **Argument Guide:**
  - To generate wind fields for a **single** environment, provide the folder to `--environment_folder` and set `--random_env` to `False`.

---

## Generating Realistic Environments with Blender & Blosm

Blender simplifies converting `.blend` files into USD and OBJ formats.

- **Convert to USD:** Export directly from Blender.
- **Convert to OBJ:** Export to USD first, then remove textures to reduce file size.
- **Important:** When exporting to OBJ, set the **Z-axis as the up-axis**.

---

## CFD Pipeline Setup

### OpenFOAM Installation

- Install the **OpenFOAM extension** in VS Code.
- The CFD automation pipeline includes some **hardcoded parameters** for efficiency. Since the file structure remains consistent, dynamic configuration is unnecessary.

### SnappyHexMesh Settings

To refine mesh settings, update the following parameters:

```c
refinementRegions
{
    refinementBox
    {
        mode inside;
        levels ((1E15 1));
    }
}

locationInMesh (1 1 1);
```

- **Large environments** (e.g., TU Delft Campus): Set refinement level to `1`.
- **Smaller environments**: Use refinement levels `2` or `3`.

---

## Manual Export of Blender Environments to USD
### Access to Building Files

The building files required for environment generation are not uploaded to GitHub due to size constraints. If you need access to these files, please contact me directly.

For manually exporting generated environments:

1. **Use `random_building_environment_creator.py`** to create procedural environments.
2. **Export to USD/OBJ via Blender utilities.**
3. Steps 1 and 2 are integrated into **`random_field.py`**.
4. **Run CFD simulation using `cfd.py`.**
5. **Generate wind fields with `wind_field.py`.**
6. Steps 4 and 5 are integrated into **`main.py`**.

---

## Running the Random Field Generator Without the Full Package

To generate wind fields without running the full package, execute the following scripts in order:

1. `random_building_environment_creator.py`
2. Export environment files (`USD` / `OBJ`) via Blender.
3. Run CFD processing using `cfd.py`.
4. Generate wind fields using `wind_field.py`.

This modular approach ensures flexibility while maintaining automation where needed.