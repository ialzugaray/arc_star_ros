# Arc*: Asynchronous Event-based Corner Detection 
This code is the reference implementation of the Arc* algorithm described in the paper  "**Asynchronous Corner Detection and Tracking for Event Cameras in Real Time**", Alzugaray & Chli, RA-L 2018. This work was developed at the [Vision for Robotics Lab](http://v4rl.ethz.ch/), [ETH Zurich](http://ethz.ch/).

Event cameras are only able to detect intensity changes in form of asynchronous events. The presented Arc* algorithm is able to detect which of these events were generated from visually salient corners in the scene. The algorithm operates solely on events and processes them individually in an asynchronous fashion. Our approach is able to handle up to several millions of events per second, achieving high-frequency feature detection and real-time performance even in challenging scenarios. 

## Video
<a href="https://youtu.be/bKUAZ7IQcf0" target="_blank"><img src="http://img.youtube.com/vi/bKUAZ7IQcf0/0.jpg" 
alt="AsynchronousCornerDetector" width="480" height="360" border="10" /></a>

## Publication
If you use this work, please cite the following [publication](https://www.research-collection.ethz.ch/handle/20.500.11850/277131): 

Ignacio Alzugaray and Margarita Chli. "**Asynchronous Corner Detection and Tracking for Event Cameras in Real Time.**" IEEE Robotics and Automation Letters (RA-L), 2018. 

    @ARTICLE{alzugaray18ral
      author={I. Alzugaray and M. Chli},
      journal={IEEE Robotics and Automation Letters},
      title={Asynchronous Corner Detection and Tracking for Event Cameras in Real Time},
      year={2018},
      volume={3},
      number={4},
      pages={3177-3184},
      doi={10.1109/LRA.2018.2849882},
      ISSN={2377-3766},
      month={Oct}}


#  Disclaimer and License
This code has been tested with ROS Kinetic on Ubuntu 16.04.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under the MIT License.

#  Instructions
We provide Arc* algorithm as a stand-alone code with minimal dependencies in the subfolder `./arc_star`, intended for high-performance benchmarking. Alternatively, you can also use the provided minimal ROS wrappers, for fast deployment, live operation or qualitative comparison. 

Requirements: 
* [Eigen 3](https://eigen.tuxfamily.org/dox/) `sudo apt-get install libeigen3-dev`

ROS-related requirements:
* [ROS Kinetic](http://wiki.ros.org/kinetic) 
* [RPG DVS ROS](https://github.com/uzh-rpg/rpg_dvs_ros) 

## Stand-alone implementation
Clone the repository and compile the project:

    $ git clone https://github.com/ialzugaray/arc_star_ros.git
    $ mkdir -p arc_star_ros/arc_star/build && cd arc_star_ros/arc_star/build
    $ cmake .. -DCMAKE_BUILD_TYPE=Release
    $ make

We provide a minimal example to process events from a plain text file. You can use the event text files from the the [Event Camera Dataset](http://rpg.ifi.uzh.ch/davis_data.html), e.g. [here](http://rpg.ifi.uzh.ch/datasets/davis/shapes_rotation.zip).

    $ ./arc_star_app_file my_dataset_folder/events.txt

You can also save the event classification into corners as a plain text file (1 event is a corner, 0 otherwise):

    $ ./arc_star_app_file my_dataset_folder/events.txt my_result_folder/classification.txt

## ROS implementation
Navigate to your initialized ROS workspace, clone and compile:

    $ cd /path/to/catkin_ws/src
    $ git clone https://github.com/ialzugaray/arc_star_ros.git
    $ cd ..
    $ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release # Assume you have already installed rpg_dvs_ros
    $ source /path/to/catkin_ws/devel/setup.bash

Connect your DAVIS camera and launch the following file:

    $ roslaunch arc_star_ros arc_star.launch

Alternatively, you can also play a rosbag file. You can use rosbags from the from the the [Event Camera Dataset](http://rpg.ifi.uzh.ch/davis_data.html), e.g. [here](http://rpg.ifi.uzh.ch/datasets/davis/shapes_rotation.bag).

    $ roslaunch arc_star_ros arc_star.launch rosbag_flag:=1 rosbag_path:=/path/to/my_bag.bag

### Notes on the Implementation
The current implementation measures the length of the arc of new elements as the number of elements that have been processed until the arc stops growing, instead of the number of elements in the arc as originally described in the  [publication](https://www.research-collection.ethz.ch/handle/20.500.11850/277131). Such subtle difference accounts for a more reliable response on edges of the scene.

# Contact
Please, create an issue if you have questions or bug reports. If you come up with any improvements, please create a pull request. Alternatively, you can also contact me at i.alzugaray@imperial.ac.uk.

