#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import JointCommand, JointCommandRequest
from std_msgs.msg import Header


class OpenManipulatorMove(object):
    def __init__(self):
        rospy.loginfo("OpenManipulatorMove INIT...Please wait.")

        # We subscribe to the joint states to have info of the system

        self.joint_states_topic_name = '/joint_states'
        self._check_join_states_ready()
        sub = rospy.Subscriber(self.joint_states_topic_name, JointState, self.joint_states_callback)


        # We start the Publisher for the positions of the joints
        self.goal_dynamixel_position_publisher = rospy.Publisher('/goal_dynamixel_position',
                                                                    JointState,
                                                                    queue_size=1)

        # Wait for the service client /joint_command to be running
        joint_command_service_name = "/joint_command"
        rospy.wait_for_service(joint_command_service_name)
        # Create the connection to the service
        self.joint_command_service = rospy.ServiceProxy(joint_command_service_name, JointCommand)

        rospy.loginfo("OpenManipulatorMove Ready!")

    def joint_states_callback(self,msg):
        """
        rosmsg show sensor_msgs/JointState
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            string[] name
            float64[] position
            float64[] velocity
            float64[] effort

        :param msg:
        :return:
        """
        self.joint_states_msg = msg

    def _check_join_states_ready(self):
        self.joint_states_msg = None
        rospy.logdebug("Waiting for "+self.joint_states_topic_name+" to be READY...")
        while self.joint_states_msg is None and not rospy.is_shutdown():
            try:
                self.joint_states_msg = rospy.wait_for_message(self.joint_states_topic_name, JointState, timeout=5.0)
                rospy.logdebug("Current "+self.joint_states_topic_name+" READY=>")

            except:
                rospy.logerr("Current "+self.joint_states_topic_name+" not ready yet, retrying ")

    def move_all_joints(self, position_array):

        # We check that the position array has the correct number of elements
        number_of_joints = len(self.joint_states_msg.name)
        if len(position_array) == number_of_joints:
            new_joint_position = JointState()

            h = Header()
            h.stamp = rospy.Time.now()  # Note you need to call rospy.init_node() before this will work
            h.frame_id = self.joint_states_msg.header.frame_id

            new_joint_position.header = h
            new_joint_position.name = self.joint_states_msg.name
            new_joint_position.position = position_array

            # These values arent used, so they dont matter really
            new_joint_position.velocity = self.joint_states_msg.velocity
            new_joint_position.effort = self.joint_states_msg.effort

            self.goal_dynamixel_position_publisher.publish(new_joint_position)
        else:
            rospy.logerr("The Array given doesnt have the correct length="+str(number_of_joints))


    def move_one_joint(self, joint_id, position, unit="rad"):
        """
        rossrv show dynamixel_workbench_msgs/JointCommand
            string unit
            uint8 id
            float32 goal_position
            ---
            bool result

        :param joint_id:
        :param position:
        :param units:
        :return:
        """
        joint_cmd_req = JointCommandRequest()
        joint_cmd_req.unit = unit
        joint_cmd_req.id = joint_id
        joint_cmd_req.position = position

        # Send through the connection the name of the object to be deleted by the service
        result = self.joint_command_service(joint_cmd_req)

        rospy.logwarn("move_one_joint went ok?="+str(result.data))

    def get_joint_names(self):
        return self.joint_states_msg.name



def movement_test():

    openman_obj = OpenManipulatorMove()

    joint_position1 = []
    joint_position2 = []

    joint_position_sequence = []
    joint_position_sequence.append(joint_position1)
    joint_position_sequence.append(joint_position2)

    # We will send each position array with a frequency of rate_freq Hz.
    rate_freq = 1.0
    rate_obj = rospy.Rate(rate_freq)

    for joint_position_array in joint_position_sequence:
        openman_obj.move_all_joints(joint_position_array)
        rate_obj.sleep()

def move_joints_test():
    """
    This is for Geting the positions of the joints without testing them
    live, which is quite dangerous!
    :return:
    """
    openman_obj = OpenManipulatorMove()
    joint_names = openman_obj.get_joint_names()
    rospy.logwarn("Starting Moving Joints GUI...")
    while not rospy.is_shutdown():
        rospy.logwarn("#######"+str(joint_names)+"#####")
        joint_id = int(raw_input("Joint ID="))
        joint_position = float(raw_input("Joint Position Radians="))
        openman_obj.move_one_joint(joint_id, joint_position, unit="rad")
        rospy.logwarn("####################")


if __name__ == "__main__":
    rospy.init_node('move_openmanipulator_node', log_level=rospy.DEBUG)
    movement_test()