/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Non-ROS C++ MoveGroup Interface
*/

#include <moveit/moveit_cpp/moveit_cpp.h>

namespace moveit
{
namespace planning_interface
{
// name of the robot description (a param name, so it can be changed externally)
const std::string MoveItCpp::ROBOT_DESCRIPTION = "robot_description";

const std::string GRASP_PLANNING_SERVICE_NAME = "plan_grasps";  // name of the service that can be used to plan grasps

MoveItCpp::MoveItCpp(const std::string& group_name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                     const ros::WallDuration& wait_for_servers)
  : MoveItCpp(MoveItCpp::Options(group_name), tf_buffer ? tf_buffer : getSharedTF(), wait_for_servers)
{
}

MoveItCpp::MoveItCpp(const std::string& group, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                     const ros::Duration& wait_for_servers)
  : MoveItCpp(group, tf_buffer, ros::WallDuration(wait_for_servers.toSec()))
{
}

MoveItCpp::MoveItCpp(const MoveItCpp::Options& opt, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                     const ros::WallDuration& wait_for_servers)
  : opt_(opt), node_handle_(opt.node_handle_), tf_buffer_(tf_buffer)
{
  // impl_ = new MoveItCppImpl(opt, tf_buffer ? tf_buffer : getSharedTF(), wait_for_servers);

  robot_model_ = opt.robot_model_ ? opt.robot_model_ : getSharedRobotModel(opt.robot_description_);
  if (!getRobotModel())
  {
    std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
                        "parameter server.";
    ROS_FATAL_STREAM_NAMED("moveit_cpp", error);
    throw std::runtime_error(error);
  }

  if (!getRobotModel()->hasJointModelGroup(opt.group_name_))
  {
    std::string error = "Group '" + opt.group_name_ + "' was not found.";
    ROS_FATAL_STREAM_NAMED("moveit_cpp", error);
    throw std::runtime_error(error);
  }

  joint_model_group_ = getRobotModel()->getJointModelGroup(opt.group_name_);

  joint_state_target_.reset(new robot_state::RobotState(getRobotModel()));
  joint_state_target_->setToDefaultValues();
  active_target_ = JOINT;
  can_look_ = false;
  can_replan_ = false;
  replan_delay_ = 2.0;
  goal_joint_tolerance_ = 1e-4;
  goal_position_tolerance_ = 1e-4;     // 0.1 mm
  goal_orientation_tolerance_ = 1e-3;  // ~0.1 deg
  allowed_planning_time_ = 5.0;
  num_planning_attempts_ = 1;
  max_velocity_scaling_factor_ = 1.0;
  max_acceleration_scaling_factor_ = 1.0;
  initializing_constraints_ = false;

  if (joint_model_group_->isChain())
    end_effector_link_ = joint_model_group_->getLinkModelNames().back();
  pose_reference_frame_ = getRobotModel()->getModelFrame();

  trajectory_event_publisher_ = node_handle_.advertise<std_msgs::String>(
      trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC, 1, false);
  attached_object_publisher_ = node_handle_.advertise<moveit_msgs::AttachedCollisionObject>(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC, 1, false);

  current_state_monitor_ = getSharedStateMonitor(robot_model_, tf_buffer_, node_handle_);

  ros::WallTime timeout_for_servers = ros::WallTime::now() + wait_for_servers;
  if (wait_for_servers == ros::WallDuration())
    timeout_for_servers = ros::WallTime();  // wait for ever
  double allotted_time = wait_for_servers.toSec();

  ROS_INFO_STREAM_NAMED("moveit_cpp", "Ready to take commands for planning group " << opt.group_name_);
}

MoveItCpp::MoveItCpp(const MoveItCpp::Options& opt, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                     const ros::Duration& wait_for_servers)
  : MoveItCpp(opt, tf_buffer, ros::WallDuration(wait_for_servers.toSec()))
{
}

MoveItCpp::~MoveItCpp()
{
  if (constraints_init_thread_)
    constraints_init_thread_->join();
}

// MoveItCpp::MoveItCpp(MoveItCpp&& other)
//   : remembered_joint_values_(std::move(other.remembered_joint_values_))
// {
// }

// MoveItCpp& MoveItCpp::operator=(MoveItCpp&& other)
// {
//   if (this != &other)
//   {
//     remembered_joint_values_ = std::move(other.remembered_joint_values_);
//   }

//   return *this;
// }

const std::string& MoveItCpp::getName() const
{
  return getOptions().group_name_;
}

const std::vector<std::string> MoveItCpp::getNamedTargets()
{
  const robot_model::RobotModelConstPtr& robot = getRobotModel();
  const std::string& group = getName();
  const robot_model::JointModelGroup* joint_group = robot->getJointModelGroup(group);

  if (joint_group)
  {
    return joint_group->getDefaultStateNames();
  }

  std::vector<std::string> empty;
  return empty;
}

robot_model::RobotModelConstPtr MoveItCpp::getRobotModel() const
{
  return getRobotModel();
}

const ros::NodeHandle& MoveItCpp::getNodeHandle() const
{
  return getOptions().node_handle_;
}

bool MoveItCpp::getInterfaceDescription(moveit_msgs::PlannerInterfaceDescription& desc)
{
  return getInterfaceDescription(desc);
}

std::map<std::string, std::string> MoveItCpp::getPlannerParams(const std::string& planner_id, const std::string& group)
{
  return getPlannerParams(planner_id, group);
}

void MoveItCpp::setPlannerParams(const std::string& planner_id, const std::string& group,
                                 const std::map<std::string, std::string>& params, bool replace)
{
  setPlannerParams(planner_id, group, params, replace);
}

std::string MoveItCpp::getDefaultPlannerId(const std::string& group) const
{
  return getDefaultPlannerId(group);
}

void MoveItCpp::setPlannerId(const std::string& planner_id)
{
  setPlannerId(planner_id);
}

const std::string& MoveItCpp::getPlannerId() const
{
  return getPlannerId();
}

void MoveItCpp::setNumPlanningAttempts(unsigned int num_planning_attempts)
{
  setNumPlanningAttempts(num_planning_attempts);
}

void MoveItCpp::setMaxVelocityScalingFactor(double max_velocity_scaling_factor)
{
  setMaxVelocityScalingFactor(max_velocity_scaling_factor);
}

void MoveItCpp::setMaxAccelerationScalingFactor(double max_acceleration_scaling_factor)
{
  setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
}

MoveItErrorCode MoveItCpp::asyncMove()
{
  return move(false);
}

MoveItErrorCode MoveItCpp::move()
{
  return move(true);
}

MoveItErrorCode MoveItCpp::asyncExecute(const Plan& plan)
{
  return execute(plan, false);
}

MoveItErrorCode MoveItCpp::execute(const Plan& plan)
{
  return execute(plan, true);
}

MoveItErrorCode MoveItCpp::plan(Plan& plan)
{
  return plan(plan);
}

moveit_msgs::PickupGoal MoveItCpp::constructPickupGoal(const std::string& object,
                                                       std::vector<moveit_msgs::Grasp> grasps, bool plan_only = false)
{
  return constructPickupGoal(object, std::move(grasps), plan_only);
}

moveit_msgs::PlaceGoal MoveItCpp::constructPlaceGoal(const std::string& object,
                                                     std::vector<moveit_msgs::PlaceLocation> locations,
                                                     bool plan_only = false)
{
  return constructPlaceGoal(object, std::move(locations), plan_only);
}

std::vector<moveit_msgs::PlaceLocation>
MoveItCpp::posesToPlaceLocations(const std::vector<geometry_msgs::PoseStamped>& poses)
{
  return posesToPlaceLocations(poses);
}

MoveItErrorCode MoveItCpp::pick(const moveit_msgs::PickupGoal& goal)
{
  return pick(goal);
}

MoveItErrorCode MoveItCpp::planGraspsAndPick(const std::string& object, bool plan_only)
{
  return planGraspsAndPick(object, plan_only);
}

MoveItErrorCode MoveItCpp::planGraspsAndPick(const moveit_msgs::CollisionObject& object, bool plan_only)
{
  return planGraspsAndPick(object, plan_only);
}

MoveItErrorCode MoveItCpp::place(const moveit_msgs::PlaceGoal& goal)
{
  return place(goal);
}

double MoveItCpp::computeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints, double eef_step,
                                       double jump_threshold, moveit_msgs::RobotTrajectory& trajectory,
                                       bool avoid_collisions, moveit_msgs::MoveItErrorCodes* error_code)
{
  moveit_msgs::Constraints path_constraints_tmp;
  return computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints_tmp, avoid_collisions,
                              error_code);
}

double MoveItCpp::computeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints, double eef_step,
                                       double jump_threshold, moveit_msgs::RobotTrajectory& trajectory,
                                       const moveit_msgs::Constraints& path_constraints, bool avoid_collisions,
                                       moveit_msgs::MoveItErrorCodes* error_code)
{
  if (error_code)
  {
    return computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints, avoid_collisions,
                                *error_code);
  }
  else
  {
    moveit_msgs::MoveItErrorCodes error_code_tmp;
    return computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints, avoid_collisions,
                                error_code_tmp);
  }
}

void MoveItCpp::stop()
{
  stop();
}

void MoveItCpp::setStartState(const moveit_msgs::RobotState& start_state)
{
  robot_state::RobotStatePtr rs;
  getCurrentState(rs);
  robot_state::robotStateMsgToRobotState(start_state, *rs);
  setStartState(*rs);
}

void MoveItCpp::setStartState(const robot_state::RobotState& start_state)
{
  setStartState(start_state);
}

void MoveItCpp::setStartStateToCurrentState()
{
  setStartStateToCurrentState();
}

void MoveItCpp::setRandomTarget()
{
  getTargetRobotState().setToRandomPositions();
  setTargetType(JOINT);
}

const std::vector<std::string>& MoveItCpp::getJointNames()
{
  return getJointModelGroup()->getVariableNames();
}

const std::vector<std::string>& MoveItCpp::getLinkNames()
{
  return getJointModelGroup()->getLinkModelNames();
}

std::map<std::string, double> MoveItCpp::getNamedTargetValues(const std::string& name)
{
  std::map<std::string, std::vector<double> >::const_iterator it = remembered_joint_values_.find(name);
  std::map<std::string, double> positions;

  if (it != remembered_joint_values_.end())
  {
    std::vector<std::string> names = getJointModelGroup()->getVariableNames();
    for (size_t x = 0; x < names.size(); ++x)
    {
      positions[names[x]] = it->second[x];
    }
  }
  else
  {
    getJointModelGroup()->getVariableDefaultPositions(name, positions);
  }
  return positions;
}

bool MoveItCpp::setNamedTarget(const std::string& name)
{
  std::map<std::string, std::vector<double> >::const_iterator it = remembered_joint_values_.find(name);
  if (it != remembered_joint_values_.end())
  {
    return setJointValueTarget(it->second);
  }
  else
  {
    if (getTargetRobotState().setToDefaultValues(getJointModelGroup(), name))
    {
      setTargetType(JOINT);
      return true;
    }
    ROS_ERROR_NAMED("moveit_cpp", "The requested named target '%s' does not exist", name.c_str());
    return false;
  }
}

void MoveItCpp::getJointValueTarget(std::vector<double>& group_variable_values) const
{
  getTargetRobotState().copyJointGroupPositions(getJointModelGroup(), group_variable_values);
}

bool MoveItCpp::setJointValueTarget(const std::vector<double>& joint_values)
{
  if (joint_values.size() != getJointModelGroup()->getVariableCount())
    return false;
  setTargetType(JOINT);
  getTargetRobotState().setJointGroupPositions(getJointModelGroup(), joint_values);
  return getTargetRobotState().satisfiesBounds(getJointModelGroup(), getGoalJointTolerance());
}

bool MoveItCpp::setJointValueTarget(const std::map<std::string, double>& variable_values)
{
  const auto& allowed = getJointModelGroup()->getVariableNames();
  for (const auto& pair : variable_values)
  {
    if (std::find(allowed.begin(), allowed.end(), pair.first) == allowed.end())
    {
      ROS_ERROR_STREAM("joint variable " << pair.first << " is not part of group " << getJointModelGroup()->getName());
      return false;
    }
  }

  setTargetType(JOINT);
  getTargetRobotState().setVariablePositions(variable_values);
  return getTargetRobotState().satisfiesBounds(getGoalJointTolerance());
}

bool MoveItCpp::setJointValueTarget(const std::vector<std::string>& variable_names,
                                    const std::vector<double>& variable_values)
{
  const auto& allowed = getJointModelGroup()->getVariableNames();
  for (const auto& variable_name : variable_names)
  {
    if (std::find(allowed.begin(), allowed.end(), variable_name) == allowed.end())
    {
      ROS_ERROR_STREAM("joint variable " << variable_name << " is not part of group "
                                         << getJointModelGroup()->getName());
      return false;
    }
  }

  setTargetType(JOINT);
  getTargetRobotState().setVariablePositions(variable_names, variable_values);
  return getTargetRobotState().satisfiesBounds(getGoalJointTolerance());
}

bool MoveItCpp::setJointValueTarget(const robot_state::RobotState& rstate)
{
  setTargetType(JOINT);
  getTargetRobotState() = rstate;
  return getTargetRobotState().satisfiesBounds(getGoalJointTolerance());
}

bool MoveItCpp::setJointValueTarget(const std::string& joint_name, double value)
{
  std::vector<double> values(1, value);
  return setJointValueTarget(joint_name, values);
}

bool MoveItCpp::setJointValueTarget(const std::string& joint_name, const std::vector<double>& values)
{
  setTargetType(JOINT);
  const robot_model::JointModel* jm = getJointModelGroup()->getJointModel(joint_name);
  if (jm && jm->getVariableCount() == values.size())
  {
    getTargetRobotState().setJointPositions(jm, values);
    return getTargetRobotState().satisfiesBounds(jm, getGoalJointTolerance());
  }

  ROS_ERROR_STREAM("joint " << joint_name << " is not part of group " << getJointModelGroup()->getName());
  return false;
}

bool MoveItCpp::setJointValueTarget(const sensor_msgs::JointState& state)
{
  return setJointValueTarget(state.name, state.position);
}

bool MoveItCpp::setJointValueTarget(const geometry_msgs::Pose& eef_pose, const std::string& end_effector_link)
{
  return setJointValueTarget(eef_pose, end_effector_link, "", false);
}

bool MoveItCpp::setJointValueTarget(const geometry_msgs::PoseStamped& eef_pose, const std::string& end_effector_link)
{
  return setJointValueTarget(eef_pose.pose, end_effector_link, eef_pose.header.frame_id, false);
}

bool MoveItCpp::setJointValueTarget(const Eigen::Isometry3d& eef_pose, const std::string& end_effector_link)
{
  geometry_msgs::Pose msg = tf2::toMsg(eef_pose);
  return setJointValueTarget(msg, end_effector_link);
}

bool MoveItCpp::setApproximateJointValueTarget(const geometry_msgs::Pose& eef_pose,
                                               const std::string& end_effector_link)
{
  return setJointValueTarget(eef_pose, end_effector_link, "", true);
}

bool MoveItCpp::setApproximateJointValueTarget(const geometry_msgs::PoseStamped& eef_pose,
                                               const std::string& end_effector_link)
{
  return setJointValueTarget(eef_pose.pose, end_effector_link, eef_pose.header.frame_id, true);
}

bool MoveItCpp::setApproximateJointValueTarget(const Eigen::Isometry3d& eef_pose, const std::string& end_effector_link)
{
  geometry_msgs::Pose msg = tf2::toMsg(eef_pose);
  return setApproximateJointValueTarget(msg, end_effector_link);
}

const robot_state::RobotState& MoveItCpp::getJointValueTarget() const
{
  return getTargetRobotState();
}

const robot_state::RobotState& MoveItCpp::getTargetRobotState() const
{
  return getTargetRobotState();
}

const std::string& MoveItCpp::getEndEffectorLink() const
{
  return getEndEffectorLink();
}

const std::string& MoveItCpp::getEndEffector() const
{
  return getEndEffector();
}

bool MoveItCpp::setEndEffectorLink(const std::string& link_name)
{
  if (getEndEffectorLink().empty() || link_name.empty())
    return false;
  setEndEffectorLink(link_name);
  setTargetType(POSE);
  return true;
}

bool MoveItCpp::setEndEffector(const std::string& eef_name)
{
  const robot_model::JointModelGroup* jmg = getRobotModel()->getEndEffector(eef_name);
  if (jmg)
    return setEndEffectorLink(jmg->getEndEffectorParentGroup().second);
  return false;
}

void MoveItCpp::clearPoseTarget(const std::string& end_effector_link)
{
  clearPoseTarget(end_effector_link);
}

void MoveItCpp::clearPoseTargets()
{
  clearPoseTargets();
}

bool MoveItCpp::setPoseTarget(const Eigen::Isometry3d& pose, const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_msg(1);
  pose_msg[0].pose = tf2::toMsg(pose);
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = ros::Time::now();
  return setPoseTargets(pose_msg, end_effector_link);
}

bool MoveItCpp::setPoseTarget(const geometry_msgs::Pose& target, const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_msg(1);
  pose_msg[0].pose = target;
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = ros::Time::now();
  return setPoseTargets(pose_msg, end_effector_link);
}

bool MoveItCpp::setPoseTarget(const geometry_msgs::PoseStamped& target, const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> targets(1, target);
  return setPoseTargets(targets, end_effector_link);
}

bool MoveItCpp::setPoseTargets(const EigenSTL::vector_Isometry3d& target, const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_out(target.size());
  ros::Time tm = ros::Time::now();
  const std::string& frame_id = getPoseReferenceFrame();
  for (std::size_t i = 0; i < target.size(); ++i)
  {
    pose_out[i].pose = tf2::toMsg(target[i]);
    pose_out[i].header.stamp = tm;
    pose_out[i].header.frame_id = frame_id;
  }
  return setPoseTargets(pose_out, end_effector_link);
}

bool MoveItCpp::setPoseTargets(const std::vector<geometry_msgs::Pose>& target, const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> target_stamped(target.size());
  ros::Time tm = ros::Time::now();
  const std::string& frame_id = getPoseReferenceFrame();
  for (std::size_t i = 0; i < target.size(); ++i)
  {
    target_stamped[i].pose = target[i];
    target_stamped[i].header.stamp = tm;
    target_stamped[i].header.frame_id = frame_id;
  }
  return setPoseTargets(target_stamped, end_effector_link);
}

bool MoveItCpp::setPoseTargets(const std::vector<geometry_msgs::PoseStamped>& target,
                               const std::string& end_effector_link)
{
  if (target.empty())
  {
    ROS_ERROR_NAMED("moveit_cpp", "No pose specified as goal target");
    return false;
  }
  else
  {
    setTargetType(POSE);
    return setPoseTargets(target, end_effector_link);
  }
}

const geometry_msgs::PoseStamped& MoveItCpp::getPoseTarget(const std::string& end_effector_link) const
{
  return getPoseTarget(end_effector_link);
}

const std::vector<geometry_msgs::PoseStamped>& MoveItCpp::getPoseTargets(const std::string& end_effector_link) const
{
  return getPoseTargets(end_effector_link);
}

namespace
{
inline void transformPose(const tf2_ros::Buffer& tf_buffer, const std::string& desired_frame,
                          geometry_msgs::PoseStamped& target)
{
  if (desired_frame != target.header.frame_id)
  {
    geometry_msgs::PoseStamped target_in(target);
    tf_buffer.transform(target_in, target, desired_frame);
    // we leave the stamp to ros::Time(0) on purpose
    target.header.stamp = ros::Time(0);
  }
}
}  // namespace

bool MoveItCpp::setPositionTarget(double x, double y, double z, const std::string& end_effector_link)
{
  geometry_msgs::PoseStamped target;
  if (hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*getTF(), getPoseReferenceFrame(), target);
  }
  else
  {
    target.pose.orientation.x = 0.0;
    target.pose.orientation.y = 0.0;
    target.pose.orientation.z = 0.0;
    target.pose.orientation.w = 1.0;
    target.header.frame_id = getPoseReferenceFrame();
  }

  target.pose.position.x = x;
  target.pose.position.y = y;
  target.pose.position.z = z;
  bool result = setPoseTarget(target, end_effector_link);
  setTargetType(POSITION);
  return result;
}

bool MoveItCpp::setRPYTarget(double r, double p, double y, const std::string& end_effector_link)
{
  geometry_msgs::PoseStamped target;
  if (hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*getTF(), getPoseReferenceFrame(), target);
  }
  else
  {
    target.pose.position.x = 0.0;
    target.pose.position.y = 0.0;
    target.pose.position.z = 0.0;
    target.header.frame_id = getPoseReferenceFrame();
  }
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  target.pose.orientation = tf2::toMsg(q);
  bool result = setPoseTarget(target, end_effector_link);
  setTargetType(ORIENTATION);
  return result;
}

bool MoveItCpp::setOrientationTarget(double x, double y, double z, double w, const std::string& end_effector_link)
{
  geometry_msgs::PoseStamped target;
  if (hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*getTF(), getPoseReferenceFrame(), target);
  }
  else
  {
    target.pose.position.x = 0.0;
    target.pose.position.y = 0.0;
    target.pose.position.z = 0.0;
    target.header.frame_id = getPoseReferenceFrame();
  }

  target.pose.orientation.x = x;
  target.pose.orientation.y = y;
  target.pose.orientation.z = z;
  target.pose.orientation.w = w;
  bool result = setPoseTarget(target, end_effector_link);
  setTargetType(ORIENTATION);
  return result;
}

void MoveItCpp::setPoseReferenceFrame(const std::string& pose_reference_frame)
{
  setPoseReferenceFrame(pose_reference_frame);
}

const std::string& MoveItCpp::getPoseReferenceFrame() const
{
  return getPoseReferenceFrame();
}

double MoveItCpp::getGoalJointTolerance() const
{
  return getGoalJointTolerance();
}

double MoveItCpp::getGoalPositionTolerance() const
{
  return getGoalPositionTolerance();
}

double MoveItCpp::getGoalOrientationTolerance() const
{
  return getGoalOrientationTolerance();
}

void MoveItCpp::setGoalTolerance(double tolerance)
{
  setGoalJointTolerance(tolerance);
  setGoalPositionTolerance(tolerance);
  setGoalOrientationTolerance(tolerance);
}

void MoveItCpp::setGoalJointTolerance(double tolerance)
{
  setGoalJointTolerance(tolerance);
}

void MoveItCpp::setGoalPositionTolerance(double tolerance)
{
  setGoalPositionTolerance(tolerance);
}

void MoveItCpp::setGoalOrientationTolerance(double tolerance)
{
  setGoalOrientationTolerance(tolerance);
}

void MoveItCpp::rememberJointValues(const std::string& name)
{
  rememberJointValues(name, getCurrentJointValues());
}

bool MoveItCpp::startStateMonitor(double wait)
{
  return startStateMonitor(wait);
}

std::vector<double> MoveItCpp::getCurrentJointValues()
{
  robot_state::RobotStatePtr current_state;
  std::vector<double> values;
  if (getCurrentState(current_state))
    current_state->copyJointGroupPositions(getName(), values);
  return values;
}

std::vector<double> MoveItCpp::getRandomJointValues()
{
  std::vector<double> r;
  getJointModelGroup()->getVariableRandomPositions(getTargetRobotState().getRandomNumberGenerator(), r);
  return r;
}

geometry_msgs::PoseStamped MoveItCpp::getRandomPose(const std::string& end_effector_link)
{
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Isometry3d pose;
  pose.setIdentity();
  if (eef.empty())
    ROS_ERROR_NAMED("moveit_cpp", "No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (getCurrentState(current_state))
    {
      current_state->setToRandomPositions(getJointModelGroup());
      const robot_model::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
        pose = current_state->getGlobalLinkTransform(lm);
    }
  }
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = getRobotModel()->getModelFrame();
  pose_msg.pose = tf2::toMsg(pose);
  return pose_msg;
}

geometry_msgs::PoseStamped MoveItCpp::getCurrentPose(const std::string& end_effector_link)
{
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Isometry3d pose;
  pose.setIdentity();
  if (eef.empty())
    ROS_ERROR_NAMED("moveit_cpp", "No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (getCurrentState(current_state))
    {
      const robot_model::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
        pose = current_state->getGlobalLinkTransform(lm);
    }
  }
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = getRobotModel()->getModelFrame();
  pose_msg.pose = tf2::toMsg(pose);
  return pose_msg;
}

std::vector<double> MoveItCpp::getCurrentRPY(const std::string& end_effector_link)
{
  std::vector<double> result;
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  if (eef.empty())
    ROS_ERROR_NAMED("moveit_cpp", "No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (getCurrentState(current_state))
    {
      const robot_model::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
      {
        result.resize(3);
        geometry_msgs::TransformStamped tfs = tf2::eigenToTransform(current_state->getGlobalLinkTransform(lm));
        double pitch, roll, yaw;
        tf2::getEulerYPR<geometry_msgs::Quaternion>(tfs.transform.rotation, yaw, pitch, roll);
        result[0] = roll;
        result[1] = pitch;
        result[2] = yaw;
      }
    }
  }
  return result;
}

const std::vector<std::string>& MoveItCpp::getActiveJoints() const
{
  return getJointModelGroup()->getActiveJointModelNames();
}

const std::vector<std::string>& MoveItCpp::getJoints() const
{
  return getJointModelGroup()->getJointModelNames();
}

unsigned int MoveItCpp::getVariableCount() const
{
  return getJointModelGroup()->getVariableCount();
}

robot_state::RobotStatePtr MoveItCpp::getCurrentState(double wait)
{
  robot_state::RobotStatePtr current_state;
  getCurrentState(current_state, wait);
  return current_state;
}

void MoveItCpp::rememberJointValues(const std::string& name, const std::vector<double>& values)
{
  remembered_joint_values_[name] = values;
}

void MoveItCpp::forgetJointValues(const std::string& name)
{
  remembered_joint_values_.erase(name);
}

void MoveItCpp::allowLooking(bool flag)
{
  allowLooking(flag);
}

void MoveItCpp::allowReplanning(bool flag)
{
  allowReplanning(flag);
}

std::vector<std::string> MoveItCpp::getKnownConstraints() const
{
  return getKnownConstraints();
}

moveit_msgs::Constraints MoveItCpp::getPathConstraints() const
{
  return getPathConstraints();
}

bool MoveItCpp::setPathConstraints(const std::string& constraint)
{
  return setPathConstraints(constraint);
}

void MoveItCpp::setPathConstraints(const moveit_msgs::Constraints& constraint)
{
  setPathConstraints(constraint);
}

void MoveItCpp::clearPathConstraints()
{
  clearPathConstraints();
}

moveit_msgs::TrajectoryConstraints MoveItCpp::getTrajectoryConstraints() const
{
  return getTrajectoryConstraints();
}

void MoveItCpp::setTrajectoryConstraints(const moveit_msgs::TrajectoryConstraints& constraint)
{
  setTrajectoryConstraints(constraint);
}

void MoveItCpp::clearTrajectoryConstraints()
{
  clearTrajectoryConstraints();
}

void MoveItCpp::setConstraintsDatabase(const std::string& host, unsigned int port)
{
  initializeConstraintsStorage(host, port);
}

void MoveItCpp::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  setWorkspace(minx, miny, minz, maxx, maxy, maxz);
}

/** \brief Set time allowed to planner to solve problem before aborting */
void MoveItCpp::setPlanningTime(double seconds)
{
  setPlanningTime(seconds);
}

/** \brief Get time allowed to planner to solve problem before aborting */
double MoveItCpp::getPlanningTime() const
{
  return getPlanningTime();
}

void MoveItCpp::setSupportSurfaceName(const std::string& name)
{
  setSupportSurfaceName(name);
}

const std::string& MoveItCpp::getPlanningFrame() const
{
  return getRobotModel()->getModelFrame();
}

const std::vector<std::string>& MoveItCpp::getJointModelGroupNames() const
{
  return getRobotModel()->getJointModelGroupNames();
}

bool MoveItCpp::attachObject(const std::string& object, const std::string& link)
{
  return attachObject(object, link, std::vector<std::string>());
}

bool MoveItCpp::attachObject(const std::string& object, const std::string& link,
                             const std::vector<std::string>& touch_links)
{
  return attachObject(object, link, touch_links);
}

bool MoveItCpp::detachObject(const std::string& name)
{
  return detachObject(name);
}

void MoveItCpp::constructMotionPlanRequest(moveit_msgs::MotionPlanRequest& goal_out)
{
  constructMotionPlanRequest(goal_out);
}

const std::shared_ptr<tf2_ros::Buffer>& MoveItCpp::getTF() const
{
  return tf_buffer_;
}

const MoveItCpp::Options& MoveItCpp::getOptionse() const
{
  return opt_;
}

const robot_model::RobotModelConstPtr& MoveItCpp::getRobotModel() const
{
  return robot_model_;
}

const robot_model::JointModelGroup* MoveItCpp::getJointModelGroup() const
{
  return joint_model_group_;
}

bool MoveItCpp::getInterfaceDescription(moveit_msgs::PlannerInterfaceDescription& desc)
{
  moveit_msgs::QueryPlannerInterfaces::Request req;
  moveit_msgs::QueryPlannerInterfaces::Response res;
  if (query_service_.call(req, res))
    if (!res.planner_interfaces.empty())
    {
      desc = res.planner_interfaces.front();
      return true;
    }
  return false;
}

std::map<std::string, std::string> MoveItCpp::getPlannerParams(const std::string& planner_id,
                                                               const std::string& group = "")
{
  moveit_msgs::GetPlannerParams::Request req;
  moveit_msgs::GetPlannerParams::Response res;
  req.planner_config = planner_id;
  req.group = group;
  std::map<std::string, std::string> result;
  if (get_params_service_.call(req, res))
  {
    for (unsigned int i = 0, end = res.params.keys.size(); i < end; ++i)
      result[res.params.keys[i]] = res.params.values[i];
  }
  return result;
}

void MoveItCpp::setPlannerParams(const std::string& planner_id, const std::string& group,
                                 const std::map<std::string, std::string>& params, bool replace = false)
{
  moveit_msgs::SetPlannerParams::Request req;
  moveit_msgs::SetPlannerParams::Response res;
  req.planner_config = planner_id;
  req.group = group;
  req.replace = replace;
  for (const std::pair<const std::string, std::string>& param : params)
  {
    req.params.keys.push_back(param.first);
    req.params.values.push_back(param.second);
  }
  set_params_service_.call(req, res);
}

std::string MoveItCpp::getDefaultPlannerId(const std::string& group) const
{
  std::stringstream param_name;
  param_name << "move_group";
  if (!group.empty())
    param_name << "/" << group;
  param_name << "/default_planner_config";

  std::string default_planner_config;
  node_handle_.getParam(param_name.str(), default_planner_config);
  return default_planner_config;
}

void MoveItCpp::setPlannerId(const std::string& planner_id)
{
  planner_id_ = planner_id;
}

const std::string& MoveItCpp::getPlannerId() const
{
  return planner_id_;
}

void MoveItCpp::setNumPlanningAttempts(unsigned int num_planning_attempts)
{
  num_planning_attempts_ = num_planning_attempts;
}

void MoveItCpp::setMaxVelocityScalingFactor(double max_velocity_scaling_factor)
{
  max_velocity_scaling_factor_ = max_velocity_scaling_factor;
}

void MoveItCpp::setMaxAccelerationScalingFactor(double max_acceleration_scaling_factor)
{
  max_acceleration_scaling_factor_ = max_acceleration_scaling_factor;
}

robot_state::RobotState& MoveItCpp::getTargetRobotState()
{
  return *joint_state_target_;
}

void MoveItCpp::setStartState(const robot_state::RobotState& start_state)
{
  considered_start_state_.reset(new robot_state::RobotState(start_state));
}

void MoveItCpp::setStartStateToCurrentState()
{
  considered_start_state_.reset();
}

robot_state::RobotStatePtr MoveItCpp::getStartState()
{
  if (considered_start_state_)
    return considered_start_state_;
  else
  {
    robot_state::RobotStatePtr s;
    getCurrentState(s);
    return s;
  }
}

bool MoveItCpp::setJointValueTarget(const geometry_msgs::Pose& eef_pose, const std::string& end_effector_link,
                                    const std::string& frame, bool approx)
{
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  // this only works if we have an end-effector
  if (!eef.empty())
  {
    // first we set the goal to be the same as the start state
    moveit::core::RobotStatePtr c = getStartState();
    if (c)
    {
      setTargetType(JOINT);
      c->enforceBounds();
      getTargetRobotState() = *c;
      if (!getTargetRobotState().satisfiesBounds(getGoalJointTolerance()))
        return false;
    }
    else
      return false;

    // we may need to do approximate IK
    kinematics::KinematicsQueryOptions o;
    o.return_approximate_solution = approx;

    // if no frame transforms are needed, call IK directly
    if (frame.empty() || moveit::core::Transforms::sameFrame(frame, getRobotModel()->getModelFrame()))
      return getTargetRobotState().setFromIK(getJointModelGroup(), eef_pose, eef, 0.0,
                                             moveit::core::GroupStateValidityCallbackFn(), o);
    else
    {
      // transform the pose into the model frame, then do IK
      bool frame_found;
      const Eigen::Isometry3d& t = getTargetRobotState().getFrameTransform(frame, &frame_found);
      if (frame_found)
      {
        Eigen::Isometry3d p;
        tf2::fromMsg(eef_pose, p);
        return getTargetRobotState().setFromIK(getJointModelGroup(), t * p, eef, 0.0,
                                               moveit::core::GroupStateValidityCallbackFn(), o);
      }
      else
      {
        ROS_ERROR_NAMED("moveit_cpp", "Unable to transform from frame '%s' to frame '%s'", frame.c_str(),
                        getRobotModel()->getModelFrame().c_str());
        return false;
      }
    }
  }
  else
    return false;
}

void MoveItCpp::setEndEffectorLink(const std::string& end_effector)
{
  end_effector_link_ = end_effector;
}

void MoveItCpp::clearPoseTarget(const std::string& end_effector_link)
{
  pose_targets_.erase(end_effector_link);
}

void MoveItCpp::clearPoseTargets()
{
  pose_targets_.clear();
}

const std::string& MoveItCpp::getEndEffectorLink() const
{
  return end_effector_link_;
}

const std::string& MoveItCpp::getEndEffector() const
{
  if (!end_effector_link_.empty())
  {
    const std::vector<std::string>& possible_eefs =
        getRobotModel()->getJointModelGroup(opt_.group_name_)->getAttachedEndEffectorNames();
    for (const std::string& possible_eef : possible_eefs)
      if (getRobotModel()->getEndEffector(possible_eef)->hasLinkModel(end_effector_link_))
        return possible_eef;
  }
  static std::string empty;
  return empty;
}

bool MoveItCpp::setPoseTargets(const std::vector<geometry_msgs::PoseStamped>& poses,
                               const std::string& end_effector_link)
{
  const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
  if (eef.empty())
  {
    ROS_ERROR_NAMED("moveit_cpp", "No end-effector to set the pose for");
    return false;
  }
  else
  {
    pose_targets_[eef] = poses;
    // make sure we don't store an actual stamp, since that will become stale can potentially cause tf errors
    std::vector<geometry_msgs::PoseStamped>& stored_poses = pose_targets_[eef];
    for (geometry_msgs::PoseStamped& stored_pose : stored_poses)
      stored_pose.header.stamp = ros::Time(0);
  }
  return true;
}

bool hasPoseTarget(const std::string& end_effector_link) const
{
  const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
  return pose_targets_.find(eef) != pose_targets_.end();
}

const geometry_msgs::PoseStamped& MoveItCpp::getPoseTarget(const std::string& end_effector_link) const
{
  const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;

  // if multiple pose targets are set, return the first one
  std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator jt = pose_targets_.find(eef);
  if (jt != pose_targets_.end())
    if (!jt->second.empty())
      return jt->second.at(0);

  // or return an error
  static const geometry_msgs::PoseStamped UNKNOWN;
  ROS_ERROR_NAMED("moveit_cpp", "Pose for end-effector '%s' not known.", eef.c_str());
  return UNKNOWN;
}

const std::vector<geometry_msgs::PoseStamped>& MoveItCpp::getPoseTargets(const std::string& end_effector_link) const
{
  const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;

  std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator jt = pose_targets_.find(eef);
  if (jt != pose_targets_.end())
    if (!jt->second.empty())
      return jt->second;

  // or return an error
  static const std::vector<geometry_msgs::PoseStamped> EMPTY;
  ROS_ERROR_NAMED("moveit_cpp", "Poses for end-effector '%s' are not known.", eef.c_str());
  return EMPTY;
}

void MoveItCpp::setPoseReferenceFrame(const std::string& pose_reference_frame)
{
  pose_reference_frame_ = pose_reference_frame;
}

void MoveItCpp::setSupportSurfaceName(const std::string& support_surface)
{
  support_surface_ = support_surface;
}

const std::string& MoveItCpp::getPoseReferenceFrame() const
{
  return pose_reference_frame_;
}

void MoveItCpp::setTargetType(ActiveTargetType type)
{
  active_target_ = type;
}

ActiveTargetType MoveItCpp::getTargetType() const
{
  return active_target_;
}

bool startStateMonitor(double wait)
{
  if (!current_state_monitor_)
  {
    ROS_ERROR_NAMED("moveit_cpp", "Unable to monitor current robot state");
    return false;
  }

  // if needed, start the monitor and wait up to 1 second for a full robot state
  if (!current_state_monitor_->isActive())
    current_state_monitor_->startStateMonitor();

  current_state_monitor_->waitForCompleteState(opt_.group_name_, wait);
  return true;
}

bool MoveItCpp::getCurrentState(robot_state::RobotStatePtr& current_state, double wait_seconds = 1.0)
{
  if (!current_state_monitor_)
  {
    ROS_ERROR_NAMED("moveit_cpp", "Unable to get current robot state");
    return false;
  }

  // if needed, start the monitor and wait up to 1 second for a full robot state
  if (!current_state_monitor_->isActive())
    current_state_monitor_->startStateMonitor();

  if (!current_state_monitor_->waitForCurrentState(ros::Time::now(), wait_seconds))
  {
    ROS_ERROR_NAMED("moveit_cpp", "Failed to fetch current robot state");
    return false;
  }

  current_state = current_state_monitor_->getCurrentState();
  return true;
}

/** \brief Convert a vector of PoseStamped to a vector of PlaceLocation */
std::vector<moveit_msgs::PlaceLocation> posesToPlaceLocations(const std::vector<geometry_msgs::PoseStamped>& poses)
{
  std::vector<moveit_msgs::PlaceLocation> locations;
  for (const geometry_msgs::PoseStamped& pose : poses)
  {
    moveit_msgs::PlaceLocation location;
    location.pre_place_approach.direction.vector.z = -1.0;
    location.post_place_retreat.direction.vector.x = -1.0;
    location.pre_place_approach.direction.header.frame_id = getRobotModel()->getModelFrame();
    location.post_place_retreat.direction.header.frame_id = end_effector_link_;

    location.pre_place_approach.min_distance = 0.1;
    location.pre_place_approach.desired_distance = 0.2;
    location.post_place_retreat.min_distance = 0.0;
    location.post_place_retreat.desired_distance = 0.2;
    // location.post_place_posture is filled by the pick&place lib with the getDetachPosture from the AttachedBody

    location.place_pose = pose;
    locations.push_back(location);
  }
  ROS_DEBUG_NAMED("moveit_cpp", "Move group interface has %u place locations", (unsigned int)locations.size());
  return locations;
}

MoveItErrorCode place(const moveit_msgs::PlaceGoal& goal)
{
}

MoveItErrorCode pick(const moveit_msgs::PickupGoal& goal)
{
}

MoveItErrorCode planGraspsAndPick(const std::string& object, bool plan_only = false)
{
  if (object.empty())
  {
    return planGraspsAndPick(moveit_msgs::CollisionObject());
  }

  PlanningSceneInterface psi;
  std::map<std::string, moveit_msgs::CollisionObject> objects = psi.getObjects(std::vector<std::string>(1, object));

  if (objects.empty())
  {
    ROS_ERROR_STREAM_NAMED("moveit_cpp", "Asked for grasps for the object '" << object
                                                                             << "', but the object could not be found");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME);
  }

  return planGraspsAndPick(objects[object], plan_only);
}

MoveItErrorCode planGraspsAndPick(const moveit_msgs::CollisionObject& object, bool plan_only = false)
{
  if (!plan_grasps_service_)
  {
    ROS_ERROR_STREAM_NAMED("moveit_cpp", "Grasp planning service '"
                                             << GRASP_PLANNING_SERVICE_NAME
                                             << "' is not available."
                                                " This has to be implemented and started separately.");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  moveit_msgs::GraspPlanning::Request request;
  moveit_msgs::GraspPlanning::Response response;

  request.group_name = opt_.group_name_;
  request.target = object;
  request.support_surfaces.push_back(support_surface_);

  ROS_DEBUG_NAMED("moveit_cpp", "Calling grasp planner...");
  if (!plan_grasps_service_.call(request, response) ||
      response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_NAMED("moveit_cpp", "Grasp planning failed. Unable to pick.");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  return pick(constructPickupGoal(object.id, std::move(response.grasps), plan_only));
}

MoveItErrorCode plan(Plan& plan)
{
  moveit_msgs::MoveGroupGoal goal;
  constructGoal(goal);
  goal.planning_options.plan_only = true;
  goal.planning_options.look_around = false;
  goal.planning_options.replan = false;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  // move_action_client_->sendGoal(goal);
}

MoveItErrorCode move(bool wait)
{
  moveit_msgs::MoveGroupGoal goal;
  constructGoal(goal);
  goal.planning_options.plan_only = false;
  goal.planning_options.look_around = can_look_;
  goal.planning_options.replan = can_replan_;
  goal.planning_options.replan_delay = replan_delay_;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  // move_action_client_->sendGoal(goal);
}

MoveItErrorCode execute(const Plan& plan, bool wait)
{
  moveit_msgs::ExecuteTrajectoryGoal goal;
  goal.trajectory = plan.trajectory_;

  // execute_action_client_->sendGoal(goal);
}

double computeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints, double step, double jump_threshold,
                            moveit_msgs::RobotTrajectory& msg, const moveit_msgs::Constraints& path_constraints,
                            bool avoid_collisions, moveit_msgs::MoveItErrorCodes& error_code)
{
  moveit_msgs::GetCartesianPath::Request req;
  moveit_msgs::GetCartesianPath::Response res;

  if (considered_start_state_)
    robot_state::robotStateToRobotStateMsg(*considered_start_state_, req.start_state);
  else
    req.start_state.is_diff = true;

  req.group_name = opt_.group_name_;
  req.header.frame_id = getPoseReferenceFrame();
  req.header.stamp = ros::Time::now();
  req.waypoints = waypoints;
  req.max_step = step;
  req.jump_threshold = jump_threshold;
  req.path_constraints = path_constraints;
  req.avoid_collisions = avoid_collisions;
  req.link_name = getEndEffectorLink();

  if (cartesian_path_service_.call(req, res))
  {
    error_code = res.error_code;
    if (res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      msg = res.solution;
      return res.fraction;
    }
    else
      return -1.0;
  }
  else
  {
    error_code.val = error_code.FAILURE;
    return -1.0;
  }
}

void stop()
{
  if (trajectory_event_publisher_)
  {
    std_msgs::String event;
    event.data = "stop";
    trajectory_event_publisher_.publish(event);
  }
}

bool attachObject(const std::string& object, const std::string& link, const std::vector<std::string>& touch_links)
{
  std::string l = link.empty() ? getEndEffectorLink() : link;
  if (l.empty())
  {
    const std::vector<std::string>& links = joint_model_group_->getLinkModelNames();
    if (!links.empty())
      l = links[0];
  }
  if (l.empty())
  {
    ROS_ERROR_NAMED("moveit_cpp", "No known link to attach object '%s' to", object.c_str());
    return false;
  }
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.id = object;
  aco.link_name.swap(l);
  if (touch_links.empty())
    aco.touch_links.push_back(aco.link_name);
  else
    aco.touch_links = touch_links;
  aco.object.operation = moveit_msgs::CollisionObject::ADD;
  attached_object_publisher_.publish(aco);
  return true;
}

bool detachObject(const std::string& name)
{
  moveit_msgs::AttachedCollisionObject aco;
  // if name is a link
  if (!name.empty() && joint_model_group_->hasLinkModel(name))
    aco.link_name = name;
  else
    aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  if (aco.link_name.empty() && aco.object.id.empty())
  {
    // we only want to detach objects for this group
    const std::vector<std::string>& lnames = joint_model_group_->getLinkModelNames();
    for (const std::string& lname : lnames)
    {
      aco.link_name = lname;
      attached_object_publisher_.publish(aco);
    }
  }
  else
    attached_object_publisher_.publish(aco);
  return true;
}

double MoveItCpp::getGoalPositionTolerance() const
{
  return goal_position_tolerance_;
}

double MoveItCpp::getGoalOrientationTolerance() const
{
  return goal_orientation_tolerance_;
}

double MoveItCpp::getGoalJointTolerance() const
{
  return goal_joint_tolerance_;
}

void MoveItCpp::setGoalJointTolerance(double tolerance)
{
  goal_joint_tolerance_ = tolerance;
}

void MoveItCpp::setGoalPositionTolerance(double tolerance)
{
  goal_position_tolerance_ = tolerance;
}

void MoveItCpp::setGoalOrientationTolerance(double tolerance)
{
  goal_orientation_tolerance_ = tolerance;
}

void MoveItCpp::setPlanningTime(double seconds)
{
  if (seconds > 0.0)
    allowed_planning_time_ = seconds;
}

double MoveItCpp::getPlanningTime() const
{
  return allowed_planning_time_;
}

void allowLooking(bool flag)
{
  can_look_ = flag;
  ROS_INFO_NAMED("moveit_cpp", "Looking around: %s", can_look_ ? "yes" : "no");
}

void allowReplanning(bool flag)
{
  can_replan_ = flag;
  ROS_INFO_NAMED("moveit_cpp", "Replanning: %s", can_replan_ ? "yes" : "no");
}

void MoveItCpp::setReplanningDelay(double delay)
{
  if (delay >= 0.0)
    replan_delay_ = delay;
}

double MoveItCpp::getReplanningDelay() const
{
  return replan_delay_;
}

void constructMotionPlanRequest(moveit_msgs::MotionPlanRequest& request)
{
  request.group_name = opt_.group_name_;
  request.num_planning_attempts = num_planning_attempts_;
  request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
  request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
  request.allowed_planning_time = allowed_planning_time_;
  request.planner_id = planner_id_;
  request.workspace_parameters = workspace_parameters_;

  if (considered_start_state_)
    robot_state::robotStateToRobotStateMsg(*considered_start_state_, request.start_state);
  else
    request.start_state.is_diff = true;

  if (active_target_ == JOINT)
  {
    request.goal_constraints.resize(1);
    request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
        getTargetRobotState(), joint_model_group_, goal_joint_tolerance_);
  }
  else if (active_target_ == POSE || active_target_ == POSITION || active_target_ == ORIENTATION)
  {
    // find out how many goals are specified
    std::size_t goal_count = 0;
    for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin();
         it != pose_targets_.end(); ++it)
      goal_count = std::max(goal_count, it->second.size());

    // start filling the goals;
    // each end effector has a number of possible poses (K) as valid goals
    // but there could be multiple end effectors specified, so we want each end effector
    // to reach the goal that corresponds to the goals of the other end effectors
    request.goal_constraints.resize(goal_count);

    for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin();
         it != pose_targets_.end(); ++it)
    {
      for (std::size_t i = 0; i < it->second.size(); ++i)
      {
        moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints(
            it->first, it->second[i], goal_position_tolerance_, goal_orientation_tolerance_);
        if (active_target_ == ORIENTATION)
          c.position_constraints.clear();
        if (active_target_ == POSITION)
          c.orientation_constraints.clear();
        request.goal_constraints[i] = kinematic_constraints::mergeConstraints(request.goal_constraints[i], c);
      }
    }
  }
  else
    ROS_ERROR_NAMED("moveit_cpp", "Unable to construct MotionPlanRequest representation");

  if (path_constraints_)
    request.path_constraints = *path_constraints_;
  if (trajectory_constraints_)
    request.trajectory_constraints = *trajectory_constraints_;
}

void constructGoal(moveit_msgs::MoveGroupGoal& goal)
{
  constructMotionPlanRequest(goal.request);
}

moveit_msgs::PickupGoal constructPickupGoal(const std::string& object, std::vector<moveit_msgs::Grasp>&& grasps,
                                            bool plan_only = false)
{
  moveit_msgs::PickupGoal goal;
  goal.target_name = object;
  goal.group_name = opt_.group_name_;
  goal.end_effector = getEndEffector();
  goal.support_surface_name = support_surface_;
  goal.possible_grasps = std::move(grasps);
  if (!support_surface_.empty())
    goal.allow_gripper_support_collision = true;

  if (path_constraints_)
    goal.path_constraints = *path_constraints_;

  goal.planner_id = planner_id_;
  goal.allowed_planning_time = allowed_planning_time_;

  goal.planning_options.plan_only = plan_only;
  goal.planning_options.look_around = can_look_;
  goal.planning_options.replan = can_replan_;
  goal.planning_options.replan_delay = replan_delay_;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
  return goal;
}

moveit_msgs::PlaceGoal constructPlaceGoal(const std::string& object,
                                          std::vector<moveit_msgs::PlaceLocation>&& locations, bool plan_only = false)
{
  moveit_msgs::PlaceGoal goal;
  goal.group_name = opt_.group_name_;
  goal.attached_object_name = object;
  goal.support_surface_name = support_surface_;
  goal.place_locations = std::move(locations);
  if (!support_surface_.empty())
    goal.allow_gripper_support_collision = true;

  if (path_constraints_)
    goal.path_constraints = *path_constraints_;

  goal.planner_id = planner_id_;
  goal.allowed_planning_time = allowed_planning_time_;

  goal.planning_options.plan_only = plan_only;
  goal.planning_options.look_around = can_look_;
  goal.planning_options.replan = can_replan_;
  goal.planning_options.replan_delay = replan_delay_;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
  return goal;
}

void MoveItCpp::setPathConstraints(const moveit_msgs::Constraints& constraint)
{
  path_constraints_.reset(new moveit_msgs::Constraints(constraint));
}

bool MoveItCpp::setPathConstraints(const std::string& constraint)
{
  if (constraints_storage_)
  {
    moveit_warehouse::ConstraintsWithMetadata msg_m;
    if (constraints_storage_->getConstraints(msg_m, constraint, robot_model_->getName(), opt_.group_name_))
    {
      path_constraints_.reset(new moveit_msgs::Constraints(static_cast<moveit_msgs::Constraints>(*msg_m)));
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

void MoveItCpp::clearPathConstraints()
{
  path_constraints_.reset();
}

void MoveItCpp::setTrajectoryConstraints(const moveit_msgs::TrajectoryConstraints& constraint)
{
  trajectory_constraints_.reset(new moveit_msgs::TrajectoryConstraints(constraint));
}

void MoveItCpp::clearTrajectoryConstraints()
{
  trajectory_constraints_.reset();
}

std::vector<std::string> MoveItCpp::getKnownConstraints() const
{
  while (initializing_constraints_)
  {
    static ros::WallDuration d(0.01);
    d.sleep();
  }

  std::vector<std::string> c;
  if (constraints_storage_)
    constraints_storage_->getKnownConstraints(c, robot_model_->getName(), opt_.group_name_);

  return c;
}

moveit_msgs::Constraints MoveItCpp::getPathConstraints() const
{
  if (path_constraints_)
    return *path_constraints_;
  else
    return moveit_msgs::Constraints();
}

moveit_msgs::TrajectoryConstraints MoveItCpp::getTrajectoryConstraints() const
{
  if (trajectory_constraints_)
    return *trajectory_constraints_;
  else
    return moveit_msgs::TrajectoryConstraints();
}

void MoveItCpp::initializeConstraintsStorage(const std::string& host, unsigned int port)
{
  initializing_constraints_ = true;
  if (constraints_init_thread_)
    constraints_init_thread_->join();
  constraints_init_thread_.reset(
      new boost::thread(boost::bind(&MoveItCppImpl::initializeConstraintsStorageThread, this, host, port)));
}

void MoveItCpp::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  workspace_parameters_.header.frame_id = getRobotModel()->getModelFrame();
  workspace_parameters_.header.stamp = ros::Time::now();
  workspace_parameters_.min_corner.x = minx;
  workspace_parameters_.min_corner.y = miny;
  workspace_parameters_.min_corner.z = minz;
  workspace_parameters_.max_corner.x = maxx;
  workspace_parameters_.max_corner.y = maxy;
  workspace_parameters_.max_corner.z = maxz;
}

void MoveItCpp::initializeConstraintsStorageThread(const std::string& host, unsigned int port)
{
  // Set up db
  try
  {
    warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
    conn->setParams(host, port);
    if (conn->connect())
    {
      constraints_storage_.reset(new moveit_warehouse::ConstraintsStorage(conn));
    }
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED("moveit_cpp", "%s", ex.what());
  }
  initializing_constraints_ = false;
}

}  // namespace planning_interface
}  // namespace moveit
