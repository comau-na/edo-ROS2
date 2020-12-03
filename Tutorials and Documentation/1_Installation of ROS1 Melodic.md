# Install ROS1 Melodic

**This tutorial assumes that you are installing ROS1 Melodic Distribution on Ubuntu 18.04**

1. Go to the [ROS Wiki Installation page](http://wiki.ros.org/ROS/Installation)
2. Click on the "Distributions" link
3. Find "ROSB Melodic Morenia" and click on the link
4. Click on the "Installation Instructions"
5. Click on Ubuntu
6. Open up a new terminal in Ubuntu and go through the installation instructions carefully
	1. Configure your Ubuntu repositories to allow "restricted", "universe", and "multiverse".
	2. Setup your sources.list
		`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
	3. Setup your keys
		`sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
	4. Install the full desktop build with the following command
		`sudo apt install ros-melodic-desktop-full`
	5. Review available packages that were installed
		`apt search ros-melodic`
7. Setup your environment
	1. To automatically add your ROS environment variables to every bash session when a new terminal is launched
		`echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`
		`source source ~/.bashrc`
	2. To manually source the distribution, use the following command
		`source /opt/ros/melodic/setup.bash`
8. Install dependencies
	1. To install "rosinstall" , "rosdep" and other command line tools you will need for this project, run the following
		`sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`
			


