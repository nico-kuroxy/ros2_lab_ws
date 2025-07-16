# launcher

A very basic ROS2 package containing all the launch files required to start each node of the system.

## How to install

- Use git clone on this repository in your robot catkin workspace.  
- Make sure you have installed the dependencies of the install_dependencies.sh script.
- Compile it with colcon and source the launch files in the install directory.
  
## How to use  

This package proposes a layered architecture to automatically start every node of the system by simply calling the one at the top of the inclusion chain : full_system.launch. This full_system.launch will include files such as full_communication.launch, full_sensors.launch, full_controllers... And each of these files are spawning the nodes belonging to their "category". Those nodes should have their respawn flag set to true, and their log space as "logs". However, these parameters can be tuned.

Speaking of which, the full_system.launch loads every parameter to the roslaunch server and handle the namespace of each node by prefixing the system's name to their own. The system's name and environment files should be stored at the top of the workspace in a dedicated config folder, where the global yaml configuration files should be as well.

### If we want to start the full_system

- To start the full system, use roslaunch with the full_system.launch launchfile.

Eg. *`ros2 launch launcher full_system.launch`*  

### If we want to start a specific set of nodes

- To start a specific set of nodes, you can call the relevant launchfile while making sure that the appropriate parameters are passed as arguments, at least the namespace. Indeed, as mentionned in the [How To Use](#how-to-use) section, the nodes are always respawning and registering their logs to the log folder, but this is tuned by the global configuration file or by in-line arguments. Since we are not reloading the entire configuration file for each sub-launchfile, we have to go with the arguments.

Eg. *`ros2 launch launcher full_communication.launch`*

- If you want to test even more specific node (and not just groups of nodes), you'll have to start them individually. For this, you can refer to the README.md of the package containing the node of interest and start it with their individual launchfile.

## Troubleshooting

- /
