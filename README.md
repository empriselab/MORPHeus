# peeling-data-collection

## Purpose

The goal of this data collection is to curate a dataset of force/torque and audio to help us classify whether peeling has been done successfully.
To do this we will be collecting four different types of data across various food items.  These types are:
- Sliding over skin
- Sliding over flesh
- Peeling skin
- Peeling flesh

The idea is that the first two types of data will be done after a peeling action has occured.  The key insight is after we peel, will feeling the region we believed we peel help us figure out if we actually peeled. One downside to doing this is that it requires two actions to figure out if we peeling. First we need to peel and then afterwards slide over the suspected peeled region.

The last two types of data will help us detect if we peeled during peeling, which means we only need one action.  

## Setup

A terminator layout has been created so that all terminator panels and commands can be quickly run. To use this layout type the following command in terminal or terminator: `terminator -l data`

This will open a terminator with every panel in the correct directory, the necessary panels will have sourced the workspace, and the conda environment should be activated.  Here is a picture of what the screen should look like:

![Screenshot from 2023-04-11 14-35-06](https://user-images.githubusercontent.com/40637887/231257675-0e28f454-a5af-4860-b0eb-ff6c57158511.png)

In case anything goes wrong here are commands that are run in each window.  All of these are in the ~/peeling_ws/ directory and have sourced devel/setup.bash (with the expection of rosbag record which is in ~/data/) starting from the top:

- `roslaunch data_collection collect_data.launch`

In the second row moving left to right are the following commands:
- `rostopic echo /audio/audio`
- `rostopic echo /forque/forqueSensor`
- `rostopic echo /key_continuous`

In the third row from left to right:
- `rosrun data_collection key_input.py`
- `rqt`

In the fourth row from left to right:
- Nothing is run currently but you should run the following: `rosbag record --all -O <fooditem_#>.bag`
- `roscore`

