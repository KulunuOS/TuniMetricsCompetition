import rospy

"""
This script assembles the top_casing on already assembled left_gear + right_gear + bottom_casing

There are two ways to assemble the top_casing

(A) top_casing_in_hand() : Pick the top_casing first and push down on semi assembled gears + bottom casing
(b) assembled_gears_in_hand() : Pick the semi_assembled gears + bottom casing and push it down on top_casing

"""
def top_casing_in_hand(detection):

    # 1. Pick top_casing
    # reach the grasp pose
    # close the gripper
    
    # 2. Assemble top_casing
    # reach the assemble pose
    # push down
    # open gripper

    return


def assembled_gears_in_hand(detection):
    
    # 1. Pick semi assembled gears
    # reach the grasp pose
    # close the gripper
    
    # 2. Assemble on  top_casing
    # reach the assemble pose
    # push down
    # open gripper
    
    return 

if __name__ == '__main__':

    rospy.init_node('top_casing_assembly')

    # Initiate grasp_client and detection_client

    # grasp_client = ?
    # detection_client = ?

    # send the grasp pose detection request (get_poses) to detection client
    object_pose_array = detection_client(get_poses)
    detection =  # in the form of a pose_msg?

    top_casing_in_hand(detection)
    #assembled_gears_in_hand(detection)


    rospy.spin()


