
# alex_gazebo

**alex_gazebo** is a ROS 2 package that provides a simulation environment for Doosan collaborative robots (cobots) using Gazebo. It includes launch files to bring up the robot in a Gazebo world and a demo script to simulate a pick-and-place task. This package simplifies simulation workflows, enabling testing and development of the cobot's functionality entirely in simulation.

## Prerequisites

Make sure you have the following installed:
- ROS 2 Humble or newer
- Gazebo (Fortress or compatible version)
- `ros_gz` bridge for ROS-Gazebo integration
- Python 3.8+
- **Docker** (as the Doosan cobot emulator is Docker-based)
  
The Doosan cobot emulator (`dsr_emulator`) is assumed to be **pre-installed** on the system for this simulation from [doosan-robot2](https://github.com/doosan-robotics/doosan-robot2/blob/31b79a03ea89460c248df952f85984bd130d8446/install_emulator.sh).

## Simulation

### 1. Starting the Doosan Cobot in Gazebo

The `dsr_bringup2_gazebo.launch.py` launch file is used to bring up the Doosan cobot in an empty Gazebo world.

To start the simulation:

```bash
ros2 launch alex_gazebo dsr_bringup2_gazebo.launch.py
```

This will launch the cobot in a Gazebo empty world environment.

### 2. Running the Pick and Place Demo

The package includes a demo script for a pick-and-place task. The demo involves the cobot picking up a simple box and moving it to a different location in the Gazebo world.

#### Steps to Run the Demo:

1. Ensure the Gazebo simulation is running (step 1).
2. Execute the pick-and-place demo script:

    ```bash
    python3 alex_gazebo/alex_gazebo/pick_place_demo.py
    ```

>**Note:** Ensure you are running the script from the `ALEX` directory, or update the script paths accordingly.

## Package Breakdown

### 1. **Config**

Doosan provided two configuration YAML files: `dsr_controller2.yaml` and `dsr_controller2_gz.yaml`. The latter is specifically for Gazebo simulations. I modified the original configuration to add control for the **gripper** via the `gripper_controller`.

The `gripper_controller` is a Gazebo forward controller type, which publishes on the `gripper_controller/command` topic. This enables control of the gripper using the `ros2_control` package, allowing us to send commands to operate the gripper.

### 3. **Launch**

The `launch` folder contains:
- `dsr_bringup2_gazebo.launch.py`: The main launch file that brings up the Doosan cobot in Gazebo.
- `dsr_gazebo.launch.py`: A supporting launch file that sets up the Gazebo world.
- `view_cobot_rviz.launch.py`: This launch file is used to load RViz with a fake position publisher, allowing us to move all the cobot's joints interactively.

### 4. **Meshes**

The `meshes/` directory contains the STL mesh files for the **AG-95** gripper. Although these meshes represent the visual and collision models of the gripper, they are **not currently used** in the simulation because the gripper’s control was simulated using geometry-based shapes with prismatic joints, as described above.

### 5. **Models**

The `models/` directory is where all the Gazebo objects (models) are defined. For example, the `simple_box` model is used in the pick-and-place demo. You can also add other objects like flasks, beakers, or any item you need for future simulations.

### 6. **Params**

This directory contains parameters used by the ROS nodes. For instance, `ros_gz_bridge_config.yaml` is a configuration file for the `ros_gz` bridge, facilitating communication between ROS 2 and Gazebo.

### 7. **ros2_control**

This directory contains files that describe how to control the cobot's joints. These files were provided by Doosan and I modified them for Gazebo compatibility. Specifically, I added controls for the gripper, including maximum limits and other joint parameters, ensuring the gripper behaves realistically in the simulation.

### 8. **XACRO**

The `xacro/` directory contains XACRO files, which are modular XML-based representations of the URDF (Unified Robot Description Format) files.

- The base URDF for the Doosan cobot is `a0509.urdf.xacro`.
- There are additional XACRO files for cameras (depth, RGB, and Intel cameras). 

For Gazebo, the depth camera plugin provided by Intel RealSense was outdated, so we used the default Gazebo plugins to simulate the **depth camera** and **RGB camera** functionality.


## Gripper Simulation (AG95)

Initially, the **AG95** gripper and its control were imported directly from the Doosan simulator. However, this did not work because the AG95 gripper lacked a suitable simulation plugin. 

To address this, we implemented a **custom workaround**:

- **Geometry-based Simulation:** A simplified version of the gripper was created using basic geometric shapes. These shapes are scaled to match the real AG95 gripper's dimensions.
  
- **Prismatic Joints:** Prismatic joints were added to simulate the movement of the gripper’s fingers. These joints mimic the linear movement required for opening and closing the gripper. 
  - The gripper control is achieved by adjusting these joints, simulating the pick-and-place operation.

- **Realistic Motion:** Although this simulation does not exactly replicate the AG95 gripper’s full mechanical behavior, it provides sufficient control for simulation purposes and closely resembles its real-world operation.

By creating this simplified model, we achieved a functional gripper for Gazebo-based simulations without needing a complete AG95-specific plugin, enabling realistic testing scenarios for robotic applications.

>**Note:** The code to control the gripper are can be read from the demo node.

## Dockerization

The `Dockerfile` and the `docker_entrypoint.sh` are located in the root of the ALEX project, allowing you to build the entire project.

### 1. Building the Docker Image

To build the Docker image, navigate to the ALEX folder and run the following command:
```bash
docker build -t ros2_alex:simulation .
```
This command builds the Docker image and tags it as ros2_alex:simulation. Once the build is complete, the image is ready for use, and you can run it on any machine with Docker installed.

### 2. Running the Simulation in Docker
There are a couple of steps to run the simulation inside Docker. First, you need to start the Doosan emulator, and then you can launch the Gazebo simulation.

#### 2.1 Start the Doosan Emulator
Before launching the Gazebo world, you need to start the Doosan dsr_emulator. Run the following command to start the emulator container:

```bash
docker run -dit --rm --privileged --name dsr01_emulator -p 12345:12345 doosanrobot/dsr_emulator:3.0.1
```
This starts the emulator container, allowing the Gazebo simulation to interact with the Doosan cobot's virtual control environment.

#### 2.2 Launch the Gazebo Simulation

Once the emulator is running, you can launch the Gazebo simulation directly from Docker by running the following command:

```bash
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix --network host -e DISPLAY --name launch_gazebo_simulation ros2_alex:simulation ros2 launch alex_gazebo dsr_bringup2_gazebo.launch.py
```

This command:

- Runs the Docker container for the Gazebo simulation with ROS 2.
- Ensures that the X11 display is properly forwarded for GUI-based simulation.
- Uses the ros2 launch command to bring up the Gazebo world with the Doosan cobot.

### 3. Running the Pick-and-Place Demo Inside Docker
Once the Gazebo simulation is up and running, you can enter the Docker container's bash shell and run the pick-and-place demo.


#### 3.1 Enter the Docker Container
To enter the running Docker container's bash shell, use the following command:

```bash
docker exec -it launch_gazebo_simulation bash
```
This opens an interactive terminal session inside the Docker container.

#### 3.2 Run the Pick-and-Place Demo
Inside the container, you can run the demo script to simulate the cobot's pick-and-place operation:

```bash
python3 /ros2_ws/src/ALEX/alex_gazebo/alex_gazebo/pick_place_demo.py
```
