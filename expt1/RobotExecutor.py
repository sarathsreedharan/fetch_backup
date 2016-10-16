import sys
import copy
import rospy
import moveit_python
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
import tf
import time
import actionlib
from moveit_python import MoveGroupInterface, PlanningSceneInterface

# DELTAS
PICKUP_DELTA_PRE = [-0.025, 0.0, 0.30]
PICKUP_DELTA_CORE = [-0.025, 0.0, 0.17]
PICKUP_DELTA_POST = [-0.025, 0.0, 0.30]
PUTDOWN_DELTA_PRE = [-0.025, 0.0, 0.30]
PUTDOWN_DELTA_CORE = [-0.025, 0.025, 0.22]
PUTDOWN_DELTA_POST = [-0.025, 0.025, 0.30]

#GRIPPER ORIENTATION
PICKUP_ORIENTATION = [-0.072, 0.661, -0.018, 0.747]
PUTDOWN_ORIENTATION = [-0.072, 0.661, -0.018, 0.747]


class RobotActionExecutor:
    def __init__(self, available_table_locations, objects, loc_marker_pos):
        self.objects = objects
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.display_trajectory_publisher = rospy.Publisher('/move_self.group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.display_trajectory_publisher = rospy.Publisher('/move_self.group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.used_table_locations = []
        self.available_table_locations = available_table_locations
        # save the intial position
        tf_listener = tf.TransformListener()
        try:
            self.initial_pos, self.initial_orient = tf_listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
        except:
            time.sleep(1)
            self.initial_pos, self.initial_orient = tf_listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
        initial_bound = 0.015
        #self.scene = moveit_python.PlanningSceneInterface('/base_link')
        #self.scene.addBox("table", 1, 1, 0.5, loc_marker_pos[self.available_table_locations[0]][0] + 0.5, loc_marker_pos[self.available_table_locations[0]][1] + 0.3, loc_marker_pos[self.available_table_locations[0]][2] - 0.3)
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        self.joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        #rospy.Subscriber('/joint_states', JointState, get_joint_state)
        self.pos = [0.353609561920166, 1.1118214007385254, -0.9501414437887573, 2.5222362207385256, -1.375872299258423, -2.724982937805176, 1.1937648574395752, -0.8393835627044678]
        initial_bound = 0.015


    def add_objects(self, curr_obj, current_state_pos):
        for obj in self.objects:
            if obj != curr_obj and obj in current_state_pos.keys():
                self.scene.addCube(obj, 0.045, current_state_pos[obj][0] - 0.0225, current_state_pos[obj][1] - 0.0225, current_state_pos[obj][2] - 0.0225)
 
#    def return_to_initial_pos(self):
#        self.group.clear_pose_targets()
#        pose_target = geometry_msgs.msg.Pose()
#        pose_target.position.x = self.initial_pos[0]
#        pose_target.position.y = self.initial_pos[1]
#        pose_target.position.z = self.initial_pos[2]
#        pose_target.orientation.x = self.initial_orient[0]
#        pose_target.orientation.y = self.initial_orient[1]
#        pose_target.orientation.z = self.initial_orient[2]
#        pose_target.orientation.w = self.initial_orient[3]
#        self.group.set_pose_target(pose_target)
#        plan = self.group.plan()
#        #x_counter = 0
#        if plan.joint_trajectory.header.frame_id == '':
#            for x_counter in range(-3, 3):
#                for y_counter in range(-3, 3):
#                    for z_counter in range(-3, 3):
#                        pose_target.position.x = self.initial_pos[0] + x_counter * 0.002
#                        pose_target.position.y = self.initial_pos[1] + y_counter * 0.002
#                        pose_target.position.z = self.initial_pos[2] + z_counter * 0.002
#                        plan = self.group.plan()
#                        if plan.joint_trajectory.header.frame_id != '':
#                            self.group.set_pose_target(pose_target)
#                            self.group.clear_pose_targets()
#                            self.group.go(plan)
#                            return True
#        else:
#            self.group.go(plan)

    def return_to_initial_pos(self):
        result = self.client.moveToJointPosition(self.joints,self.pos,0.0,max_velocity_scaling_factor=0.5)

    def close_gripper(self):
        client = actionlib.SimpleActionClient('gripper_controller/gripper_action',GripperCommandAction)
        client.wait_for_server()
        close_cmmnd = GripperCommandGoal()
        close_cmmnd.command.position = 0.045
        close_cmmnd.command.max_effort = 100
        client.send_goal(close_cmmnd)
        client.wait_for_result()

    def open_gripper(self):
        client = actionlib.SimpleActionClient('gripper_controller/gripper_action',GripperCommandAction)
        client.wait_for_server()
        close_cmmnd = GripperCommandGoal()
        close_cmmnd.command.position = 0.115
        close_cmmnd.command.max_effort = 100
        client.send_goal(close_cmmnd)
        client.wait_for_result()

    def execute_pickup(self, block, curr_positions):
        # go on top
        print "current block", block
        self.group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = curr_positions[block][0] + PICKUP_DELTA_PRE[0]
        pose_target.position.y = curr_positions[block][1] + PICKUP_DELTA_PRE[1]
        pose_target.position.z = curr_positions[block][2] + PICKUP_DELTA_PRE[2]
        pose_target.orientation.x = PICKUP_ORIENTATION[0]
        pose_target.orientation.y = PICKUP_ORIENTATION[1]
        pose_target.orientation.z = PICKUP_ORIENTATION[2]
        pose_target.orientation.w = PICKUP_ORIENTATION[3]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        stat1 = self.group.go(plan)
        if not stat1:
            return stat1
        # Move in
        self.group.clear_pose_targets()
        pose_target.position.x = curr_positions[block][0] + PICKUP_DELTA_CORE[0]
        pose_target.position.y = curr_positions[block][1] + PICKUP_DELTA_CORE[1]
        pose_target.position.z = curr_positions[block][2] + PICKUP_DELTA_CORE[2]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        stat2 = self.group.go(plan)
        if not stat2:
            return stat2
        # close gripper
        self.close_gripper()
        # Move up
        self.group.clear_pose_targets()
        pose_target.position.x = curr_positions[block][0] + PICKUP_DELTA_POST[0]
        pose_target.position.y = curr_positions[block][1] + PICKUP_DELTA_POST[1]
        pose_target.position.z = curr_positions[block][2] + PICKUP_DELTA_POST[2]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        return self.group.go(plan)
       

    def execute_putdown(self, loc_marker_positions): 
        # go on top
        self.group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        if len(self.available_table_locations) > 0:
            free_table_marker = self.available_table_locations.pop(-1)
            self.used_table_locations.append(free_table_marker)
        else:
            print "No more free table locations"
            #exit(0) # raise some exception
            return False
        print "Using the free location", free_table_marker
        pose_target.position.x = loc_marker_positions[free_table_marker][0] + PUTDOWN_DELTA_PRE[0]
        pose_target.position.y = loc_marker_positions[free_table_marker][1] + PUTDOWN_DELTA_PRE[1]
        pose_target.position.z = loc_marker_positions[free_table_marker][2] + PUTDOWN_DELTA_PRE[2]
        pose_target.orientation.x = PUTDOWN_ORIENTATION[0]
        pose_target.orientation.y = PUTDOWN_ORIENTATION[1]
        pose_target.orientation.z = PUTDOWN_ORIENTATION[2]
        pose_target.orientation.w = PUTDOWN_ORIENTATION[3]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        stat1 = self.group.go(plan)
        if not stat1:
            return stat1
        # Move in
        self.group.clear_pose_targets()
        pose_target.position.x = loc_marker_positions[free_table_marker][0] + PUTDOWN_DELTA_CORE[0]
        pose_target.position.y = loc_marker_positions[free_table_marker][1] + PUTDOWN_DELTA_CORE[1]
        pose_target.position.z = loc_marker_positions[free_table_marker][2] + PUTDOWN_DELTA_CORE[2]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        stat2 = self.group.go(plan)
        if not stat2:
            return stat2
        # close gripper
        self.open_gripper()
        # Move up
        self.group.clear_pose_targets()
        pose_target.position.x = loc_marker_positions[free_table_marker][0] + PUTDOWN_DELTA_POST[0]
        pose_target.position.y = loc_marker_positions[free_table_marker][1] + PUTDOWN_DELTA_POST[1]
        pose_target.position.z = loc_marker_positions[free_table_marker][2] + PUTDOWN_DELTA_POST[2]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        return self.group.go(plan)


    def execute_stack(self, block, current_positions):
        self.group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = current_positions[block][0] + PUTDOWN_DELTA_PRE[0]
        pose_target.position.y = current_positions[block][1] + PUTDOWN_DELTA_PRE[1]
        pose_target.position.z = current_positions[block][2] + PUTDOWN_DELTA_PRE[2]
        pose_target.orientation.x = PUTDOWN_ORIENTATION[0]
        pose_target.orientation.y = PUTDOWN_ORIENTATION[1]
        pose_target.orientation.z = PUTDOWN_ORIENTATION[2]
        pose_target.orientation.w = PUTDOWN_ORIENTATION[3]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        stat1 = self.group.go(plan)
        if not stat1:
            return stat1
        # Move in
        self.group.clear_pose_targets()
        pose_target.position.x = current_positions[block][0] + PUTDOWN_DELTA_CORE[0]
        pose_target.position.y = current_positions[block][1] + PUTDOWN_DELTA_CORE[1]
        pose_target.position.z = current_positions[block][2] + PUTDOWN_DELTA_CORE[2]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        stat2 = self.group.go(plan)
        if not stat2:
            return stat2
        # close gripper
        self.open_gripper()
        # Move up
        self.group.clear_pose_targets()
        pose_target.position.x = current_positions[block][0] + PUTDOWN_DELTA_POST[0]
        pose_target.position.y = current_positions[block][1] + PUTDOWN_DELTA_POST[1]
        pose_target.position.z = current_positions[block][2] + PUTDOWN_DELTA_POST[2]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        return self.group.go(plan)


    #def execute_unstack(self, block):
    # right now both unstack and pickup just needs the same action

    def execute_action(self, action_name, args, current_positions, loc_marker_positions):
        status = False
        if "unstack" in action_name or  "pickup" in action_name:
            #self.add_objects(args[0], current_positions)
            status = self.execute_pickup(args[0],current_positions)
        elif "putdown" in action_name:
            #self.add_objects("NAN", current_positions)
            status = self.execute_putdown(loc_marker_positions)
        elif "stack" in action_name:
            #self.add_objects(args[1], current_positions)
            status = self.execute_stack(args[1], current_positions)
        else:
            print "Unknown action ", action_name
        self.return_to_initial_pos()
        return status
        #self.scene.clear()
