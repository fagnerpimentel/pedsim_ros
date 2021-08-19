#!/usr/bin/env python
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""

import rospy
from rospkg import RosPack
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import *
from pedsim_msgs.msg import AgentStates
from std_srvs.srv import Empty, EmptyResponse


class SpawnPedsim():
    def __init__(self):
        rospack1 = RosPack()
        pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
        default_actor_model_file = pkg_path + "/models/actor_model.sdf"

        actor_model_file = rospy.get_param('~actor_model_file', default_actor_model_file)
        file_xml = open(actor_model_file)
        self.xml_string = file_xml.read()

        self.names = {}
        self.blocked = False

        rospy.loginfo("Waiting for gazebo services...")

        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        rospy.loginfo("service: spawn_sdf_model is available ....")

        rospy.wait_for_service("gazebo/delete_model")
        self.delet_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        rospy.loginfo("service: delete_model is available ....")

        rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.actor_poses_callback)

        rospy.Service('~reset', Empty, self.handle_reset_srv)

        rospy.spin()

    def handle_reset_srv(self, msg):
        er = EmptyResponse()
        self.blocked = True

        for name in self.names.keys():
            res = self.delet_model(name)
            # print(res)
            del self.names[name]

        self.blocked = False
        return er

    def actor_poses_callback(self, actors):

        if self.blocked:
            return

        for actor in actors.agent_states:
            actor_id = str( actor.id )
            if(len(self.names) > 0 and actor_id in self.names.keys()):
                continue

            actor_pose = actor.pose
            rospy.loginfo("Spawning model: actor_id = %s", actor_id)

            model_pose = Pose(Point(x= actor_pose.position.x,
                                   y= actor_pose.position.y,
                                   z= actor_pose.position.z),
                             Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )

            self.spawn_model(actor_id, self.xml_string, "", model_pose, "world")
            self.names[actor_id] = model_pose

        # rospy.signal_shutdown("all agents have been spawned !")


if __name__ == '__main__':
    try:
        rospy.init_node("spawn_pedsim_agents")
        SpawnPedsim()
    except KeyboardInterrupt:
        pass
