# CRISP

## Contact-rich robotic simulation platform with extensive geometries and contact solvers
**CRISP** is a high-fidelity physics engine tailored for complex multi-contact simulations such as tight-tolerance robotic manipulation.
This repository provides a pre-release version of CRISP for the purpose of double-blind review as part of a submission to Robotics: Science and Systems 2026.

## Requirements
- CMake >= 3.20
- C++20 compiler (recommended: GCC >= 11, Clang >= 14, Apple Clang >= 14, MSVC >= 19.30)

## Configure & Build

### Linux / macOS
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Windows
```powershell
cmake -S . -B build
cmake --build build --config Release
```

## Run Sample Codes

### Targets
- `basic`  
  Minimal example demonstrating CRISP setup and simulation loop.
- `app`  
  Interactive simulation application with OpenGL-based visualization and GUI controls.

### Linux / macOS
```bash
./build/sample/<target_name>
```

### Windows
```powershell
.\build\sample\Release\<target_name>.exe
```

## Run Evaluations

### Targets
- `01_peg_insertion_dsf`, `01_peg_insertion_sdf`  
  Peg insertion under tight tolerance using DSF-TDSF and SDF-SDF collision geometries, respectively.
- `02_bolt_nut_assembly_mesh`, `02_bolt_nut_assembly_sdf`  
  Bolt-nut assembly using Mesh-SDF and SDF-SDF collision geometries, respectively.
- `03_top_heavy_stacking`  
  Stacking of top-heavy blocks to evaluate stability under ill-conditioned configurations.
- `04_oblique_sliding`  
  A box on an inclined plane to assess frictional sliding and sticking behavior.
- `05_joint_constraints`  
  A door with a hinge joint to evaluate joint limits and friction losses.
- `06_demonstration_gear`, `06_demonstration_bolt`  
  Demonstrations of robotic manipulation tasks involving complex geometries and contact-rich interactions.

### Linux / macOS
```bash
./build/evaluations/<target_name>
```

### Windows
```powershell
.\build\evaluations\Release\<target_name>.exe
```

## License
This software is currently provided for academic peer review purposes.
Future releases will be limited to academic, non-commercial use.
See the [LICENSE](LICENSE) file for details.

## Third-Party Licenses
This software incorporates third-party components with their own licenses.
See the [NOTICE](NOTICE) file for details.
