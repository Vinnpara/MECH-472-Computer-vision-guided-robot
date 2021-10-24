# MECH-472-Computer-vision-guided-robot
***Note: Due to the Covid-19 pandemic, the robot was not built and the code was tested
on a simulation***

The objective was to build a computer vision fuided robot that is cpable of shooting a laser at an enemy robot while avoiding incoming fire form the opposing robot and obstacles on the course. An overhead camera controlled wirelessly through an Arduino nano controller and nRF24 module.

Guidance of the robot is provided by functions that use the RGB values of the differently colored circles on top of the robot to determine their centroid. The program avoids obstacles by using the relative angle between itself and the obstacle. The offensive and defensive functions of the robot are managed in a similar way.
