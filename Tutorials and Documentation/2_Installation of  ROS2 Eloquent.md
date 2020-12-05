# Install ROS2 Eloquent

**This tutorial assumes that you are installing ROS2 Eloquent Elusor Distribution on Ubuntu 18.04**

1. Go to the [ROS Wiki Installation page](https://index.ros.org/doc/ros2/Installation/Eloquent/)
2. Scroll down to "Binary Packages" and click on "Debian packages"
3. Scroll to "Setup Sources" to add teh ROS2 apt respositories to your system
	1. Run `sudo apt update && sudo apt install curl gnupg2 lsb-release`
	2. Run `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
	3. Run `sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'`
4. Install the ROS2 Packages
	1. Update your apt repository caches after setting up the repositories 
		`sudo apt update`
	2. Install deskctop version which includes ROS, RViz, demo, and tutorials 
		`sudo apt install ros-eloquent-desktop`
	3. Install ROS-Base which is a package of communication libraries, message packages, and commond line tools
		`sudo apt install ros-eloquent-ros-base`
5. Environment setup
	1. To source the ROS2 Eloquent distribution
		`source /opt/ros/eloquent/setup.bash`
