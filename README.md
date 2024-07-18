# VSCode ROS2 Workspace Template

This repository was created from [athackst/vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace). HUGE thanks to Allison Thackston for creating this template. It includes a VS Code Dev Container for ROS2 development, allowing you to develop in a consistent environment across different machines. You also don't need to install ROS2 on your host machine, as everything is contained within the Docker container.

See [how Allison Thackston develops with vscode and ros2](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how to use this workspace.

## Dev Container Features

### Style

ROS2-approved formatters are included in the IDE.  

* **c++** uncrustify; config from `ament_uncrustify`
* **python** autopep8; vscode settings consistent with the [style guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)

### Tasks

There are many pre-defined tasks, see [`.vscode/tasks.json`](.vscode/tasks.json) for a complete listing.

Take a look at [how Allison develops using tasks](https://www.allisonthackston.com/articles/vscode_tasks.html) for an idea on how I use tasks in my development.

### Debugging

This Dev Container sets up debugging for python files, gdb for cpp programs and ROS launch files.  See [`.vscode/launch.json`](.vscode/launch.json) for configuration details.

### Continuous Integration

The Dev Container also comes with basic continuous integration set up. See [`.github/workflows/ros.yaml`](/.github/workflows/ros.yaml).

## How to use this template

### Prerequisites

You should already have Docker and VSCode with the remote containers plugin installed on your system.

* [docker](https://docs.docker.com/engine/install/)
* [vscode](https://code.visualstudio.com/)
* [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Clone the repo

Now you can clone your repo as normal

### Open it in vscode

Now that you've cloned your repo onto your computer, you can open it in VSCode (File->Open Folder). 

When you open it for the first time, you should see a little popup that asks you if you would like to open it in a container.  Say yes!

![template_vscode](https://user-images.githubusercontent.com/6098197/91332551-36898100-e781-11ea-9080-729964373719.png)

If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up the container dialog

![template_vscode_bottom](https://user-images.githubusercontent.com/6098197/91332638-5d47b780-e781-11ea-9fb6-4d134dbfc464.png)

In the dialog, select "Remote Containers: Reopen in container"

VSCode will build the dockerfile inside of `.devcontainer` for you.  If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"

![template_container](https://user-images.githubusercontent.com/6098197/91332895-adbf1500-e781-11ea-8afc-7a22a5340d4a.png)

### Update the template with your code

1. Specify the repositories you want to include in your workspace in `src/ros2.repos` or delete `src/ros2.repos` and develop directly within the workspace.
2. If you are using a `ros2.repos` file, import the contents `Terminal->Run Task..->import from workspace file`
3. Install dependencies `Terminal->Run Task..->install dependencies`
4. (optional) Adjust scripts to your liking.  These scripts are used both within tasks and CI.
   * `setup.sh` The setup commands for your code.  Default to import workspace and install dependencies.
   * `build.sh` The build commands for your code.  Default to `--symlink-install`. You can also build in the command line using `colcon build --packages-select <package_name>`.
   * `test.sh` The test commands for your code. You can also run test in the command line using `colcon build --packages-select <package_name>; colcon test-result --delete-yes; colcon test --packages-select <package_name>; colcon test-result --verbose`.
5. Develop!
6. Build the code `Terminal->Run Task..->build`. This will build all of the packages in the workspace.
7. Source the code `Terminal->Run Task..->source` or source it manually `source install/setup.bash`
8. From the command line, run a launch file `ros2 launch <package> <launch_file>`. 

## Example using the Hunter SE robot

You need 6 terminals, 3 to run the simulation of the Hunter SE robot, 1 for the new node (velocity_commands_adapter), 1 for sending differential drive commands, and 1 for checking the topics.

I would have like to have a launch file that would start all of these nodes, but I didn't have time to create it.
The launch file would then be used to initialize the nodes on boot up on the robot.
The file is located in `velocity_commands_adapter/init/velocity_commands_adapter.service`. The file would be placed in `/etc/systemd/system/` and then run `sudo systemctl enable velocity_commands_adapter.service` to enable the service.

1. `source install/setup.bash; ros2 launch hunter_se_description display.launch.py`
2. `source install/setup.bash; source /usr/share/gazebo-11/setup.bash; ros2 launch hunter_se_gazebo hunter_se_empty_world.launch.py`
3. `source install/setup.bash; ros2 run rqt_robot_steering rqt_robot_steering`
4. `source install/setup.bash; ros2 launch velocity_commands_adapter differential_drive_to_ackermann_launch.py wheelbase:=0.5 cmd_vel_differential_drive_topic:=cmd_vel_differential_drive cmd_vel_ackermann_topic:=cmd_vel`
5. `ros2 topic pub /cmd_vel_differential_drive geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0}}'`
6. `ros2 topic echo /cmd_vel`

<video controls src="media/Example using the Hunter SE robot.mp4" title="Example using the Hunter SE robot"></video>

## FAQ

### WSL2

#### The gui doesn't show up

This is likely because the DISPLAY environment variable is not getting set properly.

1. Find out what your DISPLAY variable should be

      In your WSL2 Ubuntu instance

      ```
      echo $DISPLAY
      ```

2. Copy that value into the `.devcontainer/devcontainer.json` file

      ```jsonc
      	"containerEnv": {
		      "DISPLAY": ":0",
         }
      ```

#### I want to use vGPU

If you want to access the vGPU through WSL2, you'll need to add additional components to the `.devcontainer/devcontainer.json` file in accordance to [these directions](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)

```jsonc
	"runArgs": [
		"--network=host",
		"--cap-add=SYS_PTRACE",
		"--security-opt=seccomp:unconfined",
		"--security-opt=apparmor:unconfined",
		"--volume=/tmp/.X11-unix:/tmp/.X11-unix",
		"--volume=/mnt/wslg:/mnt/wslg",
		"--volume=/usr/lib/wsl:/usr/lib/wsl",
		"--device=/dev/dxg",
      		"--gpus=all"
	],
	"containerEnv": {
		"DISPLAY": "${localEnv:DISPLAY}", // Needed for GUI try ":0" for windows
		"WAYLAND_DISPLAY": "${localEnv:WAYLAND_DISPLAY}",
		"XDG_RUNTIME_DIR": "${localEnv:XDG_RUNTIME_DIR}",
		"PULSE_SERVER": "${localEnv:PULSE_SERVER}",
		"LD_LIBRARY_PATH": "/usr/lib/wsl/lib",
		"LIBGL_ALWAYS_SOFTWARE": "1" // Needed for software rendering of opengl
	},
```

### Repos are not showing up in VS Code source control

This is likely because vscode doesn't necessarily know about other repositories unless you've added them directly. 

```
File->Add Folder To Workspace
```

![Screenshot-26](https://github.com/athackst/vscode_ros2_workspace/assets/6098197/d8711320-2c16-463b-9d67-5bd9314acc7f)


Or you've added them as a git submodule.

![Screenshot-27](https://github.com/athackst/vscode_ros2_workspace/assets/6098197/8ebc9aac-9d70-4b53-aa52-9b5b108dc935)

To add all of the repos in your *.repos file, run the script

```bash
python3 .devcontainer/repos_to_submodules.py
```

or run the task titled `add submodules from .repos`

### Problems with gazebo

Try running gazebo server and gazebo client separately with verbose output.
1. Source ROS: `source install/setup.bash `
2. Source Gazebo: `source /usr/share/gazebo-11/setup.bash`. You need to source this file every time you open a new terminal.
3. Launch the gazebo server: `ros2 launch gazebo_ros gzserver.launch.py verbose:=true`
4. Launch the gazebo client: `ros2 launch gazebo_ros gzclient.launch.py verbose:=true`

A common issue is having the port already in use. You can check by running `ps` on the command line and looking for the gazebo server process. If you see it, you can kill it with `kill <pid>`

# Thoughts on using colcon vs bazel

- It was interesting trying colcon for the first time, but I found it much more tedious than bazel. The need of building before running a launch file is a bit annoying. I also found that the build times were much longer than with bazel. Defining the dependencies in the package.xml and CMakeLists.txt files was also a bit annoying. 

# TODO

- [ ] Check https://github.com/ErickKramer/ros2_with_vscode for ideas on how to improve the Dev Container.