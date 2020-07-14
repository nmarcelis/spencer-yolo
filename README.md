# Spencer People Tracker with YOLO
This package integrates the data-driven [YOLO object detector](https://pjreddie.com/darknet/yolo/) as a pedestrian detector into the Spencer People Tracking pipeline. The [ZED-YOLO wrapper](https://github.com/stereolabs/zed-yolo) from Stereolabs was integrated into ROS and then added to the Spencer tracking pipeline. Thus, the YOLO  object detector is specifically designed to work with the ZED camera. The rest of the package can also be used with other devices, e.g. a RealSense camera. 

More information about the functionalities of the Spencer People Tracking package can be found [here](spencer_people_tracking/README.md). 

With YOLO, pedestrians can be detected at close range (min. distance is approx. 0.9m) and up to a distance of several meters. Their position is computed using the depth measurements from the ZED camera. Since YOLO has a low false positive rate (depending on the chosen parameters and the weight file), it is possible to filter out false positives from the LIDAR based detector. 

The most important features of this package are listed here:
- Detect pedestrians using the ZED camera from Stereolabs and the YOLO object detector
- Use a YOLO model that was trained on the COCO dataset, which contains Pedestrians as a class. Thus, no additional training is neccessary
- Filter multiple detections of the same pedestrian using nonmax suppression
- Compute the pedestrian's Position using the pointcloud from the ZED stereo camera
- The ZED-YOLO wrapper is integrated into ROS 
- A LIDAR measures the distance to the surface of a solid body, but one is usually interested in the distance to the center of a body. The package **static calibration** corrects for that offset
- Other detectors which were already implemented in the Spencer package can still be used, also with other cameras than the ZED camera
- The YOLOv3 model is able to run at around 18 FPS on an nvidia jetson agx xavier board


Below one can see YOLO detections in an RGB image and the visualizations in rviz. Yellow boxes: LIDAR based detections. Blue boxes: YOLO detections

<p align='center' >
<a align='center' href="https://youtu.be/Abm3NSPFdbk" target="_blank">
<img src="images/tracking_5_persons_img.jpg" width="600"></img>
</a>

<a align='center'>
<img src="images/tracking_5_persons_sim.jpg" width="600"></img>
</a>
</p>


## Hardware Requirements
This package has been tested with the following Sensors:
- [Velodyne VLP16 LIDAR](https://velodynelidar.com/vlp-16.html)
- [Stereolabs ZED Stereo camera](https://www.stereolabs.com/zed/)

## Software Requirements
- Ubuntu 18.04
- Cuda 10.0
- cuDNN 7.6.5
- [ZED SDK 2.8.5 for Ubuntu 18.04 and CUDA 10.0](https://www.stereolabs.com/developers/release/#sdkdownloads_anchor). Note that SDK >= 3.0 does not work yet.
- ROS Melodic
- OpenCV >= 2.4 && <= 2.4.13.7 (follow [this](https://gist.github.com/sedovolosiy/6711123a9e5a73a6ce519e80338d0067) guide. You might have to add -D WITH_CUDA=OFF to the cmake command)

## Installation ZED SDK
First download the correct ZED SDK software. The code is tested with ZED SDK 2.8.5, but older versions might work too. Using ZED SDK >= 3.0 is not yet supported and will crash. Download ZED SDK for Cuda 10.0 and Ubuntu 18 from here: [link](https://www.stereolabs.com/developers/release/2.8/).

Once downloaded follow [this](https://www.stereolabs.com/docs/installation/linux/) guide to install the ZED SDK. The installer might ask you some questions on wheter you want to install certain dependencies. At this moment I have replied with "y" to all questions.
## Installation Cuda
First check your current Cuda version using one of the following commands:

    nvcc --version
    cat /usr/local/cuda/version.txt
If your current Cuda version is not 10.0, remove it and continue this installation guide. If it is already 10.0, go to the CuDNN installation section.

Download Cuda 10.0 from this [link](https://developer.nvidia.com/cuda-10.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=deblocal) but do not yet follow the installation instructions found on that website. Instead follow this guide from Nvidia to set up everything correctly before installing: [link](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html). Once you have installed Cuda, download the patch found also on the Cuda download page and open it using the Ubuntu software installer. If the patch is installed, make sure you follow the post installation steps found in the Nvidia installation guide.

## Installation CuDNN
To install CuDNN you first need to verify if you have not already installed it. Use this to verify it:

    cat /usr/include/cudnn.h | grep CUDNN_MAJOR -A 2

If it says CUDNN_MAJOR 7, CUDNN_MINOR 6, CUDNN_PATCHLEVEL 5, then you have already installed the correct CuDNN version. If another version is present, remove this version. Go to the Nvidia CuDNN download site and download CuDNN 7.6.5 for Cuda 10.0: [link](https://developer.nvidia.com/rdp/cudnn-download). Login with your Nvidia account and accept the ethical AI option (if you consent). You have to download the "cuDNN Runtime Library for Ubuntu18.04 (Deb)" and "cuDNN Developer Library for Ubuntu18.04 (Deb)" files. The "cuDNN Code Samples and User Guide for Ubuntu18.04 (Deb)" is optional. You can install all the Debian files by opening it using the Ubuntu Software Installer. Verify the correct installation using the command above.

## Installation OpenCV

    sudo apt-get install build-essential

    sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    
    sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
    sudo apt update
    sudo apt install libjasper1 libjasper-dev
    
    sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
    
    cd ~ 
    git clone https://github.com/opencv/opencv.git
    cd opencv
    git checkout 2.4
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -D WITH_CUDA=OFF ..
    make all -j$(nproc) # Uses all machine cores    
    sudo make install
    
    pkg-config --modversion opencv      # Verify installation of correct version




## Installation
Create Catkin workspace, clone the repo and install dependencies:

    sudo apt-get install python-catkin-tools
    mkdir -p catkin-ws/src
    cd catkin-ws
    catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cd src
    git clone https://github.com/nmarcelis/spencer-yolo.git
    rosdep update
    rosdep install -r --from-paths . --ignore-src
    sudo apt-get install libsvm-dev

Compile Darknet: A fork from @AlexeyAB is [used](https://github.com/AlexeyAB/darknet)

    cd spencer_people_tracking_yolo/zed-yolo/libdarknet
    make -j4

Build the ROS packages

    cd catkin_ws
    catkin build -c -s
    source devel/setup.bash


## Weight files
There are two different weight files one can use. The lightweight YOLOv3 model is able to run at around 18 FPS on an nvidia jetson agx xavier board. The heavier YOLOv3 model is slower but has a slightly better detection accuracy. It is recommended to use the lightweight model. 

Download them from [here](https://pjreddie.com/darknet/yolo/)
and place them in here:

    spencer_people_tracking_yolo/zed-yolo/libdarknet/weights


## Running the Detection-Tracking-Pipeline 

### Launch the YOLO Pedestrian detection module
There are two different launch files available, one for either of the two weight files:

lighweight model:
    
    roslaunch yolo_pedestrian_detector pedestrian_detector_tiny.launch

normal model:

    roslaunch yolo_pedestrian_detector pedestrian_detector.launch

### Launch the Spencer people tracking pipeline
In the launch file below, one can choose the pedestrian detector one wants to use. YOLO is the default detector.
One must publish the camera's and LIDAR's tf. This can be done in the launch file if it is not done anywhere else already (set to False by default). 

    roslaunch spencer_people_tracking_launch tracking_with_yolo.launch


## References
[Link](https://drive.google.com/file/d/17gYQlm1KNR1uVZGOEhCawVPUrMDU37_R/view?usp=sharing) to Semester Thesis.
