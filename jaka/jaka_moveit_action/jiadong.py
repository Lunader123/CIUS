import rospy, sys
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped

# from moveit_commander.move_group import MoveGroupCommander

if __name__ == "__main__":
    rospy.init_node('jiadong_gogogo')
    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.MoveGroupCommander('arm')
    # pub_goal = rospy.Publisher("/rviz/moveit/update_goal_state", ,queue_size=10)
    # pub_start = rospy.Publisher("/rviz/moveit/update_start_state", ,queue_size=10)
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)
    arm.allow_replanning(True)
    end_effector_link = arm.get_end_effector_link()
    print(end_effector_link)
    # target_pose = PoseStamped()
    # target_pose.header.frame_id = reference_frame
    # target_pose.header.stamp = rospy.Time.now()     
    # target_pose.pose.position.x = 0.5
    # target_pose.pose.position.y = 0.3
    # target_pose.pose.position.z = 1.3
    # target_pose.pose.orientation.x = 1
    # target_pose.pose.orientation.y = 1
    # target_pose.pose.orientation.z = 1
    # target_pose.pose.orientation.w = 1
    # arm.set_pose_target(target_pose)
    # arm.go()
    arm.set_named_target('zero_pose')
    arm.go()
    rospy.sleep(2)
    xyz = [0.2,0,0.8]
    arm.set_position_target(xyz, end_effector_link)
    # arm.go()
    traj = arm.plan()
    arm.execute(traj)
    rospy.sleep(2)
    arm.set_named_target('zero_pose')
    arm.go()