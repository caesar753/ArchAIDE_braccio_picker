#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import time


from gazebo_msgs.msg import LinkStates, ModelState
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import SetModelState
from custom_msgs.msg import matrix

## END_SUB_TUTORIAL
import numpy as np
import scipy.optimize
import cv2
import json

import InvKin #as Arm3Link

##FROM MOVEIT_TUTORIALS
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

THETA_EXT = 0.27
THETA_RET = np.pi/4

L_FUDGE = 0.08

Z_MAX_SIDE = -0.03
Z_MAX_DOWN = 0
Z_MIN = -0.045

CLOSE_ENOUGH = 0.02
DEFAULT_ROT = 0

S_SIDE_MAX = 0.4
S_SIDE_MIN = 0.161
S_TOP_MAX = 0.29

def cart2pol(x, y):
    """helper, convert cartesian to polar coordinates"""
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    """helper,convert polar to cartesian"""
    x = rho*np.cos(phi)
    y = rho*np.sin(phi)
    return(x, y)

def get_other_angles(theta_shoulder):
  """helper, converting some angles"""
  theta_wrist = theta_shoulder + np.pi/2
  theta_elbow = np.pi/2 - 2*theta_shoulder
  return theta_wrist, theta_elbow

##FROM MOVEIT_TUTORIALS
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True  

class BraccioPoseGoal(object):
  """BraccioPoseGoalInterface"""
  def __init__(self, node_handle):
    super(BraccioPoseGoal, self).__init__()

    self.nh = node_handle

    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('braccio_xy_bb_target', anonymous=True)
    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.linkstate_callback)
    self.targets_list = []
    self.i = 0
    # Subscriber to /targets topic with matrix message
    # self.target_matrix = rospy.Subscriber("/targets", matrix, self.callback_matrix)
    #sleep else ROS cannot get the robot state and the matrix msg
    # rospy.sleep(1)
    #unregister target_matrix subscriber
    # self.target_matrix.unregister()  

    group_name = "braccio_arm"
    self.move_group = moveit_commander.MoveGroupCommander(group_name)
    self.move_group.set_planner_id("RRTConnectkConfigDefault")
    self.gripper_group = moveit_commander.MoveGroupCommander("braccio_gripper")

    self.homography = None

    self.kinematics = InvKin.Arm3Link()

    self.random_pose = PoseStamped()

    self.display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
    )

    ## Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    self.scene = moveit_commander.PlanningSceneInterface()
  def calibrate(self):
    joint_home = self.move_group.get_current_joint_values()
    print(f'joint values for home is {joint_home}')

    """scan a series of points and record points in gazebo and robot frames"""
    src_pts = []
    dst_angs = []
    # mouseX, mouseY, r_ = self.get_link_position(['kuka::base_link'])
    links = [['kuka::mount'], ['kuka::left_gripper_link'],['kuka::right_gripper_link']]
    # mouseX, mouseY, r_ = self.get_link_position(links[0])
    mouseX, mouseY, r_ = self.get_link_position(['kuka::mount'])
    src_pts.append([mouseX,mouseY])

    self.gripper_middle()
    N = 8
    phi_min = np.pi/6
    phi_max = np.pi - np.pi/6
    for i in range(2,N):
      self.go_to_raise()
      if i % 2 == 0:
        rand_phi = phi_min + i*(phi_max-phi_min)/N
        theta_shoulder = THETA_RET
      else:
        theta_shoulder = THETA_EXT
      theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
      rand_targ = [rand_phi,theta_shoulder,theta_elbow, theta_wrist]
      self.go_to_j(j0=rand_phi,j1=theta_shoulder,j2=theta_elbow,j3=theta_wrist)
      mouseX, mouseY, r_ = self.get_link_position(['kuka::left_gripper_link','kuka::right_gripper_link'])
      src_pts.append([mouseX,mouseY])
      # mouseX, mouseY, r_ = self.get_link_position('kuka::right_gripper_link')
      # src_pts.append([mouseX,mouseY])
      dst_angs.append(rand_targ)
    with open('calibration.json', 'w') as f:
      json.dump({'src_pts':src_pts,'dst_angs':dst_angs},f)
    self.load_calibrate()
    self.go_to_up()
    # self.move_group.go(joint_home, wait=True)
    # print(f'joint values for home is {joint_home}')
    self.go_start_position()
    return

  def load_calibrate(self):
    """load mapping points from gazebo to robot frame, estimate l and L, generate homography map"""
    try:
      with open('calibration.json', 'r') as f:
        calib = json.load(f)
      src_pts = calib['src_pts']
      dst_angs = calib['dst_angs']

      s_ret_pts = src_pts[1::2]
      s_ext_pts = src_pts[2::2]
      arr = np.array(s_ret_pts)-np.array(s_ext_pts)
      self.L = np.sqrt((arr*arr).sum(axis=1)).mean()/(np.cos(THETA_EXT)-np.cos(THETA_RET))
      arr = np.array(s_ret_pts)-np.array(src_pts[0])
      l1 = np.sqrt((arr*arr).sum(axis=1)).mean() - self.L*np.cos(THETA_RET)
      arr = np.array(s_ext_pts)-np.array(src_pts[0])
      l2 = np.sqrt((arr*arr).sum(axis=1)).mean() - self.L*np.cos(THETA_EXT)
      self.l = (l1+l2)/2

      dst_pts = [[0,0]]
      for i in range(len(dst_angs)):
        phi = dst_angs[i][0]
        rho = self.L*np.cos(dst_angs[i][1]) + self.l
        x, y = pol2cart(rho, phi)
        dst_pts.append([x,y])

      src_pts = np.array(src_pts)
      dst_pts = np.array(dst_pts)

      h, status = cv2.findHomography(src_pts, dst_pts)
      self.homography = h

      self.kinematics = InvKin.Arm3Link(L=[self.L/2,self.L/2,self.l+L_FUDGE])
      print('calibration loaded.')
      print('estimated l = ' + str(self.l))
      print('estimated L = ' + str(self.L))
      cv2.destroyAllWindows()
    except:
      print('calibration.json not in current directory, run calibration first')
      self.calibrate()

  def linkstate_callback(self, data):
    """callback to get link location for cube from gazebo"""
    try:
      self.linkstate_data = data
      # rospy.loginfo("linkstate_data received") #+ str(self.linkstate_data))
      # print(self.linkstate_data)
      # return(self.linkstate_data)
    except ValueError:
      pass

##ADDED FROM REPAIR MOVEIT_TEST
  def go_to_pos(self, target_pose):
        # joint_goal = self.move_group.get_current_joint_values()

        # move_arm_to_pose_req = MoveArmToPoseRequest()
        # move_arm_to_pose_req.arm = ARM_ENUM.ARM_2.value
        # move_arm_to_pose_req.target_pose = target_pose
        # self.move_to_pose(ARM_ENUM.ARM_2, target_pose)
        pose_start = self.move_group.get_current_pose()

        # target_int = Pose()
        # target_int.position.x = target_pose.position.x
        # target_int.position.y = target_pose.position.y
        # target_int.position.z = target_pose.position.z
        # target_int.orientation.w = target_pose.orientation.w

        self.move_group.set_pose_target(target_pose)

         ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        # # get plan
        # _, plan, _, _ = self.move_group.plan()
        # print(plan)
        # plan = self.move_group.plan()
        # self.move_group.execute(plan)

        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        # display_trajectory.trajectory_start = self.robot.get_current_state()
        # display_trajectory.trajectory.append(plan)
        # # Publish
        # self.display_trajectory_publisher.publish(display_trajectory)

        # len_points = len(plan.joint_trajectory.points)
        # print(len_points)

        # # print points
        # self.nh.loginfo(f'points in plan: {len(plan.joint_trajectory.points)}')

        # if len_points == 0:
        #     self.nh.logwarn("No plan found")
        #     return False

    
  
  def get_planning_infos(self):
    # We can get the name of the reference frame for this robot:
    planning_frame = self.move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = self.move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = self.robot.get_group_names()
    print("============ Available Planning Groups:", self.robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(self.robot.get_current_state())
    print("")


  def random(self):
    random_pose = self.move_group.get_random_pose()
    # print(type(random_pose))
    # print(random_pose)
    self.random_pose = random_pose
    print(f'Random pose is \n \
      {self.random_pose}')
    return self.random_pose
  
  def planning_pose_random(self):
    print(f"End effector link is {self.move_group.get_end_effector_link()}")

    pose_start = self.move_group.get_current_pose()
    print(f"pose_start is\n \
            {pose_start}")

    self.move_group.set_planning_time(10)
    self.move_group.set_num_planning_attempts(5)
    self.move_group.set_goal_tolerance(0.5)

    print(f" Random pose goal is \n \
      {self.random_pose}")

    self.move_group.set_pose_target(self.random_pose)

    success = self.move_group.go(wait=True)
    print(success)
    self.move_group.stop()

    self.move_group.clear_pose_targets()

  def planning_pose_example(self):
    print(f"End effector link is {self.move_group.get_end_effector_link()}")

    pose_start = self.move_group.get_current_pose()
    print(f"pose_start is\n \
            {pose_start}")

    self.move_group.set_planning_time(10)
    self.move_group.set_num_planning_attempts(5)
    self.move_group.set_goal_tolerance(0.5)
    
    pose_goal = Pose()
    pose_goal.position.z = pose_start.pose.position.z + 0.5
    pose_goal.orientation.w = 1.0
    pose_goal.orientation = pose_start.pose.orientation

    print(f" pose goal is {pose_goal}")

    self.move_group.set_pose_target(pose_goal)
    
    success = self.move_group.go(wait=True)
    print(success)

    self.move_group.stop()
    
    self.move_group.clear_pose_targets()



  # def planning_pose_example(self):
  #   # self.move_group.set_end_effector_link("link_5")
  #   print(f"End effector link is {self.move_group.get_end_effector_link()}")

  #   pose_start = self.move_group.get_current_pose()
  #   print(f"pose_start is\n \
  #         {pose_start}")

  #   self.move_group.set_planning_time(10)
  #   self.move_group.set_num_planning_attempts(5)
  #   self.move_group.set_goal_tolerance(1.5)
  #   pose_goal = Pose()
  #   # pose_goal.header = pose_start.header
  #   # # pose_goal.position = pose_start.pose.position

  #   # pose_goal.position.x = pose_start.pose.position.x + 0.1
  #   # pose_goal.position.y = pose_start.pose.position.y + 0.1
  #   pose_goal.position.z = pose_start.pose.position.z + 0.1
  #   # pose_goal.orientation.w = pose_start.pose.orientation.w + 0.1
  #   pose_goal.orientation.w = 1.0

  #   print(f" pose goal is {self.random_pose}")

  #   # pose_goal_2 = Pose()
  #   # pose_goal_2.position.z = pose_start.pose.position.z - 0.1
  #   # pippo = self.move_group.set_pose_target(pose_goal)
    
  #   # self.move_group.set_goal_tolerance(0.2)
  #   self.move_group.set_pose_target(self.random_pose)
  #   # # print(f"set_pose_target is {pippo}")    

  #   # # `go()` returns a boolean indicating whether the planning and execution was successful.
  #   success = self.move_group.go(wait=True)
  #   print(success)
  #   # # Calling `stop()` ensures that there is no residual movement
  #   self.move_group.stop()
  #   # # It is always good to clear your targets after planning with poses.
  #   # # Note: there is no equivalent function for clear_joint_value_targets().
  #   self.move_group.clear_pose_targets()

  #   # # current_pose = prova_move.get_current_pose().pose
  #   # # return all_close(pose_goal, current_pose, 0.01)

  #   # get plan
  #   # _, plan, _, _ = self.move_group.plan()
  #   # print(plan)
  #   # # plan = self.move_group.plan()
  #   # self.move_group.execute(plan)

  def set_standard_position(self, position):
    self.move_group.set_goal_tolerance(0.2)
    #set position of braccio to all_up from SRDF
    self.move_group.set_named_target(position)
    success = self.move_group.go(wait=True)
    print(success)
    self.move_group.stop()

  def set_joint(self):
     
    pose_start = self.move_group.get_current_pose()
    print(f"pose_start is\n \
          {pose_start}")


    pose_goal = Pose()
    # pose_goal.header = pose_start.header
    # # pose_goal.position = pose_start.pose.position

    # pose_goal.position.x = pose_start.pose.position.x + 0.1
    # pose_goal.position.y = pose_start.pose.position.y + 0.1
    pose_goal.position.z = pose_start.pose.position.z + 0.15
    # pose_goal.orientation.w = pose_start.pose.orientation.w + 0.1
    pose_goal.orientation.w = 1.0

    print(f" pose goal is {pose_goal}")

    # target = Pose()
    # target = target_pose
    self.move_group.set_joint_value_target(pose_goal, arg2="link_5", arg3=True)

    # print(pippo)