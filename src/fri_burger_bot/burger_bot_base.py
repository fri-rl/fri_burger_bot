#!/usr/bin/env python
import rospy


from tf.transformations import quaternion_from_euler, euler_matrix
import tf2_ros
import geometry_msgs.msg

import threading

import numpy as np

from sawyer_pykdl import sawyer_kinematics

from sensor_msgs.msg import JointState, PointCloud2

import ros_numpy

class BurgerBotBase(object):

    def __init__(self):

        self._camera_pose_thread = threading.Thread(target=self._publish_camera_transform)

        self._target_thread = threading.Thread(target=self._compute_target)
        self._items_thread = threading.Thread(target=self._publish_items)
        self._filtered_point_cloud_thread = threading.Thread(target=self._publish_filtered_point_cloud)
        self._ghost_thread = threading.Thread(target=self._publish_ghost)


        self._io_thread = threading.Thread(target=self._io_run)

        self._br = tf2_ros.TransformBroadcaster()

        self._kin = sawyer_kinematics()

        self._joint_state = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self._joint_states_sub = rospy.Subscriber("/alexei/joint/states", JointState, self._joint_states_cb)
        self._ghost_states_pub = rospy.Publisher("/alexei/ghost/joint/states", JointState, queue_size=10)


        self._point_cloud = None
        self._point_cloud_sub = rospy.Subscriber("/kinect1/qhd/points/", PointCloud2, self._point_cloud_cb)

        self._filtered_point_cloud_pub = rospy.Publisher('/alexei/filtered_point_cloud2', PointCloud2, queue_size=10)


        possible_items = ['burger','tomato','cheese','salat','toast1','toast2']
        self._point_cloud_pubs = dict(zip(possible_items, [rospy.Publisher('/items/{}/point_cloud2'.format(item), PointCloud2, queue_size=10) for item in possible_items]))

        self.items = None
        self.item_points = None
        self.item_clouds = None
        self.item_names = None

        self._des_joint_state = None

    def filter_point_cloud(self, point_cloud):

        return point_cloud

    def _publish_filtered_point_cloud(self):

        rate = rospy.Rate(10)
        while self.keep_running:
            if self._point_cloud is not None:
                cloud = ros_numpy.point_cloud2.array_to_pointcloud2(ros_numpy.point_cloud2.get_array_from_xyzrgb(self._point_cloud), rospy.Time.now(), 'alexei/base')
                self._filtered_point_cloud_pub.publish(cloud)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                # print(err)
                pass

    def get_joint_state(self):
        return self._joint_state

    def get_joint_limits(self):
        return np.array([-3.0503,-3.8095,-3.0426,-3.0439,-2.9761,-2.9761,-4.7124]),np.array([3.0503,2.2736,3.0426,3.0439,2.9761,2.9761,4.7124])

    def get_jacobian(self, joint_state=None):
        if joint_state is None:
            joint_state = self._joint_state
        return self._kin.jacobian(joint_state)

    def get_endeffector_pose(self, joint_state=None):
        if joint_state is None:
            joint_state = self._joint_state
        return self._kin.forward_position_kinematics(joint_state, rot_type='rpy')

    def start(self):

        self.keep_running = True
        self._camera_pose_thread.start()
        self._target_thread.start()
        self._items_thread.start()
        self._ghost_thread.start()
        self._filtered_point_cloud_thread.start()
        self._io_thread.start()

    def stop(self):

        self.keep_running = False
        self.join()

    def join(self):
        self._camera_pose_thread.join()
        self._target_thread.join()
        self._items_thread.join()
        self._ghost_thread.join()
        self._filtered_point_cloud_thread.join()


    def _publish_ghost(self):

        rate = rospy.Rate(100)
        while self.keep_running:

            if self._des_joint_state is not None:
                des_joint_state = self._des_joint_state
                des_joint_state_msg = JointState()
                des_joint_state_msg.header.stamp = rospy.Time.now()
                des_joint_state_msg.name = ["head_pan", "right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6", "torso_t0"]
                des_joint_state_msg.position = [0.0]*9
                des_joint_state_msg.position[1:8] = des_joint_state
                des_joint_state_msg.velocity = [0.0]*9
                des_joint_state_msg.effort = [0.0]*9
                self._ghost_states_pub.publish(des_joint_state_msg)

            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                # print(err)
                pass

    def _joint_states_cb(self, msg):

        if 'right_j0' in msg.name:
            self._joint_state = np.array(msg.position[1:8])

    def _point_cloud_cb(self, msg):
        point_cloud = ros_numpy.point_cloud2.pointcloud2_to_xyzrgb_array(msg)

        transform = self.get_camera_pose()
        R = euler_matrix(transform[3],transform[4],transform[5])[:3,:3]
        point_cloud[:,0:3] = np.dot(R,point_cloud[:,0:3].T).T + transform[0:3]

        self._point_cloud = self.filter_point_cloud(point_cloud)


    def _publish_frame(self, frame_pose, frame_name):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "alexei/base"
        t.child_frame_id = frame_name
        t.transform.translation.x = frame_pose[0]
        t.transform.translation.y = frame_pose[1]
        t.transform.translation.z = frame_pose[2]
        if len(frame_pose) == 3:
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = (0.0,0.0,0.0,1.0)
        elif len(frame_pose) == 6:
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quaternion_from_euler(frame_pose[3],frame_pose[4],frame_pose[5])
        elif len(frame_pose) == 7:
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = frame_pose[3],frame_pose[4],frame_pose[5],frame_pose[6]
        else:
            raise ValueError("pose has to be either 3d, 6d or 7d but was {} {}".format(frame_pose.shape,len(frame_pose)))

        self._br.sendTransform(t)

    def _publish_camera_transform(self):

        rate = rospy.Rate(100)
        while self.keep_running:
            camera_pose = self.get_camera_pose()
            self._publish_frame(camera_pose, "kinect1_rgb_optical_frame")

            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                # print(err)
                pass

    def _compute_target(self):

        joint_state_history_max = 50
        joint_state_history = []


        rate = rospy.Rate(100)
        while self.keep_running:

            joint_state_history.append(self.get_joint_state())
            if len(joint_state_history) > joint_state_history_max:
                del joint_state_history[0]

            if len(joint_state_history) == joint_state_history_max:
                intersection = self.compute_z_intersection(joint_state_history)
                if intersection is not None:
                    intersection = np.append(intersection, [1.0, 0.0, 0.0, 0.0])
                    self._publish_frame(intersection, "intersection")

                    if self.items is not None:
                        target_item, target_name = self.identify_target_item(self.items, self.item_names, intersection[0:3])
                        target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                        target[0:3] = target_item[0:3]
                        self._publish_frame(target, "target")

            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                joint_state_history = []
                pass

    def _publish_items(self):

        rate = rospy.Rate(100)
        while self.keep_running:

            stamp = rospy.Time.now()
            if self.items is not None:

                for item, name, points, cloud in zip(self.items, self.item_names, self.item_points, self.item_clouds):
                    self._publish_frame(item[0:3],"detected_{}".format(name))

                    if cloud:
                        cloud.header.stamp = stamp
                        self._point_cloud_pubs[name].publish(cloud)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass



    def _io_run(self):


        while self.keep_running:


            item_names, item_idx = (None,None) if self.item_names is None else (", ".join(['"{}"'.format(name) for name in self.item_names]), " 1-{}".format(len(self.item_names)))
            help = """
            Enter one of the following:
               i : identify items.
                   known items [{}]
            {} : compute required configuration for respective item.
               q : quit
            """.format(item_names, item_idx)
            cmd = raw_input("\n\n{}\n>>> ".format(help))

            if cmd == "q":
                self.stop()
            elif cmd == "i":
                self._call_identify_items()
            else:
                try:
                    target_idx = int(cmd)
                    print(target_idx,len(self.item_names))
                    if (target_idx > 0) and (target_idx <= len(self.item_names)):
                        self._call_compute_target_configuration(self.items[target_idx-1,:])
                        continue
                except Exception as err:
                    print(err)
                    pass

                print("Sorry, I didn't get that.")

    def _call_identify_items(self):

        items, item_points = self.detect_items(self._point_cloud)
        item_names = self.identify_items(items)

        clouds = []
        for points in item_points:
            clouds.append(ros_numpy.point_cloud2.array_to_pointcloud2(ros_numpy.point_cloud2.get_array_from_xyzrgb(points), rospy.Time.now(), 'alexei/base'))

        self.items, self.item_names, self.item_points, self.item_clouds = items, item_names, item_points, clouds



    def _call_compute_target_configuration(self, target_item):
        target_pose = np.array([0.0, 0.0, 0.0, 0.5*np.pi, 0.0, 0.0])
        target_pose[0:3] = target_item[0:3]
        des_joint_state = self.compute_target_configuration(self.get_joint_state(), target_pose)
        self._des_joint_state = des_joint_state


    #####################################
    # To be implemented by the students #
    #####################################

    def get_camera_pose(self):
        raise NotImplementedError()

    def detect_items(self, data):
        raise NotImplementedError()

    def identify_items(self, items):
        raise NotImplementedError()

    def compute_z_intersection(self, joints, z=-0.065 ):
        raise NotImplementedError()

    def identify_target_item(self, items, names, intersection):
        raise NotImplementedError()

    def compute_target_configuration(self, joints, des_cart):
        raise NotImplementedError()
