# vision_info_server

![Developed By OUXT Polaris](img/logo.png "Logo")

| *master* | *develop* |
|----------|-----------|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/vision_info_server.svg?branch=master)](https://travis-ci.org/OUXT-Polaris/vision_info_server)|[![Build Status](https://travis-ci.org/OUXT-Polaris/vision_info_server.svg?branch=develop)](https://travis-ci.org/OUXT-Polaris/vision_info_server)|

Vision info server read XML file and set it to the rosparam.
After that, vision_info_server publish vision_msgs/VisionInfo topic

# nodes

vision_info_server_node

## input topics

no

## output topics

~/vision_info

## input param

~/xml_path

## output param

~/class_meta_info