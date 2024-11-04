# Human Model Control in Gazebo Through ROS

This project simulates human model control using ROS and Gazebo. It includes a human model with configurable ROS controllers for movement and spawns various objects in a Gazebo world for collaborative and dynamic environment simulation.

![human random control](https://github.com/user-attachments/assets/aa531964-0038-4228-a950-e03b27624a87)


## Features

- **Human Model Simulation**: A detailed URDF-based human model with ROS controllers for position and movement.
- **Gazebo Integration**: Objects such as cubes, sensors, and guards to simulate a collaborative environment.
- **ROS Controller Configurations**: Easily customizable ROS controllers for the human model.

## Getting Started

### Prerequisites

Ensure you have the following installed:

- ROS (tested with Foxy and Humble)
- Gazebo
- `ros_control` and `controller_manager` packages

### Installation

1. Clone this repository:
   ```bash
   cd src
   git clone https://github.com/ammar-n-abbas/human_model_gazebo_ros_control
   ```

2. Build the workspace:
   ```bash
   colcon build
   source devel/setup.bash
   ```

### Directory Structure

```
human_model_gazebo_ros_control/
├── config/
│   └── ros_controllers.yaml  # Configuration for ROS controllers
├── urdf/
│   └── human_model.urdf      # URDF file for human model
├── launch/
│   └── demo_gazebo_ros_control.launch  # Launch file for human model and controllers
│   └── gazebo.launch  # Launch file for gazebo
└── scripts/
    └── human_random_control.py  # Python file for human random control
```

## Usage

1. **Launching the Human Model and Controllers**:

   Run the following command to start the simulation:
   ```bash
   ros2 launch <your-ws-name> demo_gazebo_ros_control.launch
   ```

   Run the following command to start the random control movement:
   ```bash
   python ./scripts/human_random_control.py
   ```

2. **Configuration**:
   - To modify controller settings, edit `config/ros_controllers.yaml`.
   - URDF changes for the human model can be made in `urdf/human_model.urdf`.

### Parameters

- `human_description_file`: Path to the human model URDF file.
- `controller_config_file`: Path to the YAML file defining ROS controllers.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

This project has emanated from research conducted with the financial support of the EU Commission Recovery and Resilience Facility under the Science Foundation Ireland Future Digital Challenge Grant Number 22/NCF/FD/10929.
