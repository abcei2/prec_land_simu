# QUADCOPTER PRECISION LANDING SIMULATION

Let's test precision landing with gazebo+ardupilot+**python3.5**, yes python3.5 this is a convenient and faster way to do so  
because the goal is to load virtual gimbal cámera topic loaded in the example world **iris_runaway** of **ardupilot_gazebo**  
which deploya simulation of a drone who can be controlled by **QGroundControl**.  
  
## Installation 
```bash
sudo apt install git gitk git-gui curl
```
### QGroundControl
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
Then logout and login again.  
  
In the folder you want to save QGC:  
```bash
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
sudo chmod +x ./QGroundControl.AppImage
```
### Ardupilot  
```bash
git clone https://github.com/ArduPilot/ardupilot  
cd ardupilot  
git submodule update --init --recursive  
Tools/environment_install/install-prereqs-ubuntu.sh -y  
```
**Reload the path (log-out and log-in to make permanent):**  
```
 ~/.profile  
 ```
 **(optional) Add autotest folder to path to exec sim_vehicle.py from any path**  
 At the end of  ~/.bashrc file:  
 ```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest  
export PATH=/usr/lib/ccache:$PATH  
 ```
### Gazebo  
**ON UBUNTU**, with ubuntu 20.04 will get gazebo 11.0.  
```bash
curl -sSL http://get.gazebosim.org | sh  
```
### ardupilot_gazebo  
```bash
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
### Python3.5, and packages pygazebo, mavlink, opencv-contrib 
Installing Python3.5 without mess up some previous installed version  
```bash
sudo apt-get install build-essential checkinstall
sudo apt-get install libreadline-gplv2-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev
cd /usr/src
wget https://www.python.org/ftp/python/3.5.9/Python-3.5.9.tgz
sudo tar xzf Python-3.5.9.tgz
cd Python-3.5.9
sudo ./configure --enable-optimizations
sudo make altinstall
```
Installing python packages.  
```bash
sudo python3.5 -m pip install -U pip
sudo python3.5 -m pip install pygazebo tqdm pymavlink Pillow asyncio numpy PyYaml opencv-contrib-python
```

## REF
http://gazebosim.org/tutorials?tut=install_ubuntu
