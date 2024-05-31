<h1>Bonehead</h1>

![Bonehead, in all its' glory.](assets/BONEHEAD_ISO.jpg?raw=true "Bonehead")

<h2>Northern Illinois University AY 23-24 Senior Design Project Team 10:
  Summonable Construction Delivery Robot</h2>

<p>The Bonehead project is a first-year senior design capstone project done at Northern Illinois University in AY 2023 to 2024.
  The team members consist of Ben Crowninshield, Tyler Stewart and Kevin Lewis. The project consists of a src directory and arduino sketches.
  This comes from the selected hardware in use for Bonehead. We use an Arduino MKR WAN 1310 development board for hardware control, which interfaces
  with a Raspberry Pi 4B used to host the ROS2. In this configuration, the repository, when cloned, is the intended ROS2 workspace. If the directory
  level changes, some issues have been known to occur, especially when building the packages at different directories other than the repository level,
  with package build overlays. To avoid this problem, it is highly recommended to clone this repository and use colcon on the top level. A COLCON IGNORE
  file will avoid any build issues with the C++ code in the arduino directory.</p>

<p>Some Key Features:</p>
<ul>
<li>Inverse Kinematics Model and corresponding RViz2 Model.</li>
<li>Naive Gait Planning Algorithm, implementing the Bezier Curve approach for footpath generation.</li>
<li>Simple manual control using bluetooth controller and PyGame.</li>
</ul>

![Bonehead, in all its' glory.](assets/Full_Arm_Assembly_Left_Animation.gif?raw=true "Bonehead")

Build Requirements:
<ul>
  <li> Ubuntu 22.04.4 LTS (Humble)</li>
  <li> C++17</li>
  <li> Python 3.6</li>
</ul>

<h2>Installation</h2>

<h3>1. Dependencies</h3>
<p>
  From the ROS2 workspace, resolve all depedencies for the project.
</p>

```
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
<h3>2. Build</h3>
<p>
  Once the dependencies are installed, use colcon to begin building.
</p>

```
colcon build
```
<h3>3. Source</h3>
<p>
  Lastly, source the bash install script to have access to the launch files from bash.
</p>

```
source install/setup.bash
```
<p>
  Alternatively, you can permanently source the bash setup file with the following command.
  Replace "your_workspace_path" with the path to your clone of the repository.
</p>

```
echo "source your_workspace_path/install/setup.bash" >> ~/.bashrc
```
