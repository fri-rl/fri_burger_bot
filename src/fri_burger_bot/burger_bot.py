#!/usr/bin/env python

import numpy as np
from fri_burger_bot.burger_bot_base import BurgerBotBase
# you shouldn't import any other modules
# all modules you need are imported


class BurgerBot(BurgerBotBase):
    # In order to solve the project properly, you should call any method/member
    # starting with an underscore, e.g., self._joint_state
    # However, the follwoing methods should occur (at least once each)
    # somewhere in your implementations
    #
    # self.get_endeffector_pose(joint_state): returns a 7 dof
    #



    # We know that the base of the robot is localted at
    # [-0.107, -0.429, 1.448] (x,y,z translations)
    # and [-3.085, 0.933, 1.629] (in roll, pitch, yaw : extrinsic rotations around x,y,z )
    # but this function needs to return a 6d vector containing
    # the pose of the camera in the robot base frame in [x,y,z, roll, pitch, yaw]
    #
    # Debug Hint: the rosbag contains frames called "burger_dummy", "cheese_dummy" etc.
    # If you chose the right transformation they should be roughly at the center of the respective item
    #
    # Ignore the "ROS time moved backwards" Error, that is unfortunately normal when replaying bags in a loop
    def get_camera_pose(self):

        # This transformation is wrong, replace it with the correct one.
        return np.array([0.107, 0.429, -1.448, 3.085, -0.933, -1.629])


    # This method gets a Nx6 numpy array where each row
    # represents a datapoint consisting of the points location
    # in the robot frame and its color values: [x,y,z,r,g,b].
    #
    # The method should return a Mx6 numpy array where each row
    # represents the centroid of an object on the table along with
    # the r,g,b values of the most common color.
    # In addition the method should return a M sized list of N_1x6,...,N_Mx6
    # np.arrays, where each array contains the datapoints belonging to the respective item
    #
    # r,g,b values are given as floats from 0.0 to 1.0
    #
    # Hint: It makes sense to filter the data points for their position before
    # any other computation, e.g. not all datapoints are on or part of the table!
    # Also not all datapoints belong to the items...
    #
    # Debug Hint: you can check if your found items make sense by changing the topic of the 'PointCLoud2' plugin in rviz
    def detect_items(self, data):

        # this computation is pointless and just so something can be returned.
        # replace it with something usefull
        idxBurger = (data[:,0] >= 0.50) & (data[:,0] <= 0.65) & (data[:,1] >=  0.05) & (data[:,1] <=  0.15) & (data[:,2] >= -0.2) & (data[:,2] <= 0.0)
        idxCheese = (data[:,0] >= 0.50) & (data[:,0] <= 0.65) & (data[:,1] >= -0.05) & (data[:,1] <=  0.05) & (data[:,2] >= -0.2) & (data[:,2] <= 0.0)
        idxTomato = (data[:,0] >= 0.65) & (data[:,0] <= 0.80) & (data[:,1] >= -0.05) & (data[:,1] <=  0.05) & (data[:,2] >= -0.2) & (data[:,2] <= 0.0)
        idxSalat  = (data[:,0] >= 0.65) & (data[:,0] <= 0.80) & (data[:,1] >=  0.05) & (data[:,1] <=  0.15) & (data[:,2] >= -0.2) & (data[:,2] <= 0.0)
        idxToast1 = (data[:,0] >= 0.50) & (data[:,0] <= 0.65) & (data[:,1] >= -0.15) & (data[:,1] <= -0.05) & (data[:,2] >= -0.2) & (data[:,2] <= 0.0)
        idxToast2 = (data[:,0] >= 0.65) & (data[:,0] <= 0.80) & (data[:,1] >= -0.15) & (data[:,1] <= -0.05) & (data[:,2] >= -0.2) & (data[:,2] <= 0.0)
        items = np.array([
        [0.57,  0.10, -0.06, 0.0, 1.0, 1.0],
        [0.57,  0.00, -0.06, 0.0, 0.0, 1.0],
        [0.72,  0.00, -0.06, 1.0, 1.0, 0.0],
        [0.72,  0.10, -0.06, 1.0, 0.0, 1.0],
        [0.57, -0.10, -0.06, 1.0, 0.0, 0.0],
        [0.72, -0.10, -0.06, 0.0, 1.0, 0.0]])

        points = [data[idxBurger,:], data[idxCheese,:], data[idxTomato,:], data[idxSalat,:], data[idxToast1,:], data[idxToast2,:]]

        # Do not change the return line!
        return (items, points)


    # This method gets a Mx6 numpy array where each row
    # represents the center of an item and it's most frequent color.
    #
    # This methods should return a list of M strings, where the ith string
    # identifies which object is present in the ith row of the input array.
    #
    # Hint: think about which features are representative of the task and which ones are not
    # Possible strings are: "burger", "cheese", "tomato", "salat", "toast1", "toast2"
    def identify_items(self, items):

        # this computation is pointless and just so something can be returned.
        # replace it with something usefull
        item_names = ["burger", "cheese", "tomato", "salat", "toast1", "toast2"]
        item_names = [item_names[i%6] for i in range(items.shape[0])]

        # Do not change the return line!
        return item_names


    # This method gets a 50x7 numpy array where each row
    # corresponds to a joint configuration over the last 50 time steps
    # with joints[0,:] being the oldest and joints[-1,:] being the
    # most current joint configuration
    #
    # The method should return the x, y and z (as a numpy array) values of the tool tip for a given z-value
    # based on the given joint configurations.
    # Alternatively if no reasonable itersection can be computed the method should return: None
    #
    def compute_z_intersection(self, joints, z=-0.065 ):


        # this computation is pointless and just so something can be returned.
        # replace it with something usefull
        x = 0.5
        y = 0.0

        # Do not change the return line!
        return np.array([x,y,z])


    # This method gets a Mx6 numpy array (items) and a list of strings of length M (names).
    # Each row of items represents the center of an item and it's most frequent color.
    # names contains the corresponding names
    # In addition the method gets a point in 3d space given as a numpy array (intersection)
    #
    # The method should return the row and the name of the item
    # that is the intended target based on intersection
    def identify_target_item(self, items, names, intersection):

        # this computation is pointless and just so something can be returned.
        # replace it with something usefull
        ridx = np.random.randint(0,len(names))
        target_item = items[ridx,:]
        target_name = names[ridx]

        # Do not change the return line!
        return (target_item,target_name)


    # This method gets a 7d numpy array (joints) corresponding to the current joint configuration
    # of the robot and additionally a 6d numpy array (des_cart) representing a desired pose in cartesian space.
    # des_cart is given as [x,y,z,roll,pitch,yaw] ,i.e., translations in x,y,z and extrinsic rotations around x,y,z
    #
    # The method should return a configuration for which the end effector is at the des_cart
    def compute_target_configuration(self, joints, des_cart):

        # this computation is pointless and just so something can be returned.
        # replace it with something usefull
        lower_limit, upper_limit = self.get_joint_limits()
        des_joints = np.random.uniform(lower_limit, upper_limit)

        # Do not change the return line!
        return (des_joints)
