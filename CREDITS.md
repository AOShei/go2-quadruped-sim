# Credits and Attribution

This project builds upon and incorporates work from several open-source projects and sources. We gratefully acknowledge the following contributions:

## CHAMP Quadruped Controller

**Project**: CHAMP (CHAMPionship quadruped controller framework)
**Author**: Juan Miguel Jimeno
**Copyright**: 2019-2020 Juan Miguel Jimeno
**License**: BSD-3-Clause
**Repository**: https://github.com/chvmp/champ
**Usage**: The core quadruped kinematics, gait generation, and body control algorithms are heavily based on the CHAMP framework. The code has been extensively modified and adapted for ROS 2 Jazzy compatibility and specifically tuned for the Unitree Go2 robot's kinematic structure.

**Location in this project**: `src/unitree_go2_controller/include/champ/`

**Key modifications**:
- Updated for ROS 2 Jazzy API compatibility
- Adapted kinematics for Unitree Go2's 3-joint leg configuration
- Modified coordinate transformations to match Go2 joint conventions
- Integrated with ros2_control framework
- Tuned gait parameters for Go2's mass and dimensions

## BasicLinearAlgebra (BLA) Library

**Author**: tomstewart89
**Copyright**: 2019 tomstewart89
**License**: MIT License
**Repository**: https://github.com/tomstewart89/BasicLinearAlgebra
**Usage**: Lightweight linear algebra library used for matrix and vector operations in kinematics calculations.

**Location in this project**: `src/unitree_go2_controller/include/champ/bla/`

## XMLRPC Helpers

**Author**: PAL Robotics, S.L.
**Copyright**: 2013 PAL Robotics, S.L.
**License**: Modified BSD License
**Usage**: Utility functions for handling ROS parameter server operations.

**Location in this project**: `src/unitree_go2_controller/include/champ/utils/xmlrpc_helpers.h`

## Unitree Go2 Robot Assets

**Source**: Unitree Robotics
**License**: BSD-3-Clause (as distributed by Unitree)
**Usage**: Robot mesh files (.dae format) for visual and collision geometry, and kinematic parameters based on official Unitree Go2 specifications.

**Location in this project**: `src/unitree_go2_description/meshes/`

**Note**: The URDF/xacro robot description has been created based on publicly available specifications and mesh files from Unitree Robotics, with adaptations for Gazebo Harmonic simulation and ros2_control integration.

---

## Project Maintainer

**Name**: Andrew O'Shei
**Email**: andrewoshei@gmail.com
**License**: BSD-3-Clause
**Year**: 2024-2025

**Original contributions**:
- ROS 2 Jazzy integration and wrapper code
- Gazebo Harmonic simulation setup
- Nav2 integration and configuration
- SLAM Toolbox integration
- Custom launch files and configurations
- Robot description files adapted for Gazebo Harmonic
- Documentation and examples

---

## License Summary

This project is licensed under the **BSD-3-Clause License** (see LICENSE file). All incorporated third-party components are used in compliance with their respective licenses:

- **CHAMP**: BSD-3-Clause (compatible, same license)
- **BLA**: MIT License (compatible, permissive)
- **XMLRPC Helpers**: Modified BSD (compatible, permissive)
- **Unitree Assets**: BSD-3-Clause (compatible, same license)

All licenses require attribution, which is provided in this file and in copyright headers within source files.

---

## Acknowledgments

Special thanks to:
- **Juan Miguel Jimeno** for creating the excellent CHAMP framework that made this project possible
- **Unitree Robotics** for producing the Go2 robot and providing specifications
- The **ROS 2** and **Gazebo** communities for their outstanding open-source tools
- The **Nav2** and **SLAM Toolbox** teams for robust navigation software

---

For questions about licensing or attribution, please contact andrewoshei@gmail.com or open an issue on the project repository.
