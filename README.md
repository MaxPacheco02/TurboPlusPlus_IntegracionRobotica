# Turbo++ IntegracionRobotica TE3003B.501

## Configuration Manual

### Prerequisites

To run the contents of this repository, make sure you have the following:

1. Ubuntu 22.04

If you do not have Ubuntu installed, you can look at this [Disk partition for Dual Booting with Windows and Ubuntu 22][dual-boot].

2. ROS 2 Humble

If you do not have this distribution installed, [read the following installation guide][ros2].

3. GZ SIM Garden

If you do not have this version of Gz sim installed, [read the following installation guide][gz].

### Running this repository
```Shell
# Clone the repository and initialize submodules
git clone https://github.com/MaxPacheco02/TurboPlusPlus_IntegracionRobotica.git
cd TurboPlusPlus_IntegracionRobotica
git submodule update --init --recursive

# Build the ROS2 workspace
colcon build

# Start running the nodes!

```

## Team members:
### *Andrés Carrizales - A00832679*
### *Daniel Castañon - A01284991*
### *Demian Marin - A01368643*
### *Max Pacheco - A01552369*
### *Neil Hernandez - A01284713*


[dual-boot]: https://www.youtube.com/watch?v=QKn5U2esuRk
[ros2]: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
[gz]: https://gazebosim.org/docs/garden/install_ubuntu
