sudo apt-get install python3.7
sudo rm /usr/bin/python3
sudo ln -s /usr/bin/python3.7 /usr/bin/python3

sudo apt-get update
sudo apt-get install v4l-utils
sudo apt-get install libcanberra-gtk-module
v4l2-ctl --list-devices
v4l2-ctl --all
v4l2-ctl --help-all
v4l2-ctl --list-formats
v4l2-ctl -d /dev/video0 --list-formats-ext
pip install v4l2-python3

sudo apt-get install python3-pip
sudo -H pip3 install --upgrade pip
python3.7 -m pip install matplotlib

python3.7 -m pip install pips

pip3 install --upgrade pip 

sudo apt-get install python3.7 python3.7-dev
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install build-essential
sudo apt-get install zlib1g-dev libsqlite3-dev tk-dev
sudo apt-get install libssl-dev openssl
sudo apt-get install libffi-dev
pip3 install flake8 



git clone https://github.com/alliedvision/V4L2Viewer.git
sudo apt install qt5-default cmake
cd V4L2Viewer
mkdir build && cd build
cmake ..
make
fdc;

git clone https://github.com/alliedvision/examples.git
./BasicDemo --device /dev/video0

sudo apt-get update
apt-get install libcanberra-gtk-module
sudo apt-get install libgstreamer1.0-0
sudo apt install gstreamer1.0-plugins-ugly gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
sudo apt-get install gstreamer1.0-tools


https://github.com/JetsonHacksNano/installVSCode.git
https://github.com/tliron/8812au.git
./installVSCode.sh

sudo apt-get install build-essential dkms 


v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=GREY --set-ctrl exposure=33300,gain=110,bypass_mode=0 --stream-mmap --stream-count=1 --stream-to=test.mp4

ls /usr/bin/python*
python --version
sudo su
update-alternatives --install /usr/bin/python python /usr/bin/python3 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 2


sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2
##update-alternatives: using /usr/bin/python3.7 to provide /usr/bin/python3 (python3) in auto mode


sudo vim ~/.bashrc
alias python3='/usr/bin/python3.7'
alias pip3='python3.7 -m pip'
source ~/.bashrc

sudo apt-get install openssh-server
sudo apt install openssh-server
sudo systemctl enable ssh
## OR enable and start the ssh service immediately ##
sudo systemctl enable ssh --now
sudo systemctl status ssh
sudo systemctl enable ssh
sudo systemctl start ssh
sudo systemctl start ssh


git clone https://github.com/alliedvision/VimbaPython.git


