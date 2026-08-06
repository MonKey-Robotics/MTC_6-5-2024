/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Michael 'v4hn' Goerner */

#include "execute_task_solution_capability.h"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/message_checks.h>
#include <fmt/format.h>
#include <chrono>
#include <thread>

namespace {

// TODO: move to moveit::core::RobotModel
const moveit::core::JointModelGroup* findJointModelGroup(const moveit::core::RobotModel& model,
                                                         const std::vector<std::string>& joints) {
	std::set<std::string> joint_set(joints.begin(), joints.end());

	const std::vector<const moveit::core::JointModelGroup*>& jmgs = model.getJointModelGroups();

	for (const moveit::core::JointModelGroup* jmg : jmgs) {
		const std::vector<std::string>& jmg_joints = jmg->getJointModelNames();
		std::set<std::string> jmg_joint_set(jmg_joints.begin(), jmg_joints.end());

		// return group if sets agree on all active joints
		if (std::includes(jmg_joint_set.begin(), jmg_joint_set.end(), joint_set.begin(), joint_set.end())) {
			std::set<std::string> difference;
			std::set_difference(jmg_joint_set.begin(), jmg_joint_set.end(), joint_set.begin(), joint_set.end(),
			                    std::inserter(difference, difference.begin()));
			unsigned int acceptable = 0;
			for (const std::string& diff_joint : difference) {
				const moveit::core::JointModel* diff_jm = model.getJointModel(diff_joint);
				if (diff_jm->isPassive() || diff_jm->getMimic() || diff_jm->getType() == moveit::core::JointModel::FIXED)
					++acceptable;
			}
			if (difference.size() == acceptable)
				return jmg;
		}
	}

	return nullptr;
}
}  // namespace

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_visualization.execute_task_solution");

namespace move_group {

ExecuteTaskSolutionCapability::ExecuteTaskSolutionCapability() : MoveGroupCapability("ExecuteTaskSolution") {}

void ExecuteTaskSolutionCapability::initialize() {
	// configure the action server

	 auto node = context_->moveit_cpp_->getNode();

	as_ = rclcpp_action::create_server<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
	    context_->moveit_cpp_->getNode(), "execute_task_solution",
	    ActionServerType::GoalCallback(std::bind(&ExecuteTaskSolutionCapability::handleNewGoal, this,
	                                             std::placeholders::_1, std::placeholders::_2)),
	    ActionServerType::CancelCallback(
	        std::bind(&ExecuteTaskSolutionCapability::preemptCallback, this, std::placeholders::_1)),
	    ActionServerType::AcceptedCallback(
	        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTaskSolutionAction>>& goal_handle) {
		        last_goal_future_ =
		            std::async(std::launch::async, &ExecuteTaskSolutionCapability::execCallback, this, goal_handle);
	        }));

      // Initialize end_effector_link_
	node->get_parameter_or("move_group.plan_execution.end_effector_link", end_effector_link_, std::string("shear_tip"));
	// Initialize publisher for /planned_path
	planned_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
	RCLCPP_INFO(LOGGER, "Initialized /planned_path publisher");
}

nav_msgs::msg::Path ExecuteTaskSolutionCapability::trajectoryToPath(
    const robot_trajectory::RobotTrajectory& trajectory,
    const std::string& link_name,
    const std::string& frame_id) const

{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id; // Use the planning frame (e.g., world or base_link)
  auto node = context_->moveit_cpp_->getNode();
  path.header.stamp = node->get_clock()->now();

  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
    const moveit::core::RobotState& state = trajectory.getWayPoint(i);

    // Compute forward kinematics for the specified link
    const Eigen::Isometry3d& transform = state.getGlobalLinkTransform(link_name);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = transform.translation().x();
    pose.pose.position.y = transform.translation().y();
    pose.pose.position.z = transform.translation().z();
    pose.pose.orientation.x = Eigen::Quaterniond(transform.rotation()).x();
    pose.pose.orientation.y = Eigen::Quaterniond(transform.rotation()).y();
    pose.pose.orientation.z = Eigen::Quaterniond(transform.rotation()).z();
    pose.pose.orientation.w = Eigen::Quaterniond(transform.rotation()).w();
    path.poses.push_back(pose);
  }
  return path;

}

void ExecuteTaskSolutionCapability::execCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTaskSolutionAction>>& goal_handle) {
	auto result = std::make_shared<moveit_task_constructor_msgs::action::ExecuteTaskSolution::Result>();

	const auto& goal = goal_handle->get_goal();
	if (!context_->plan_execution_) {
		result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
		goal_handle->abort(result);
		return;
	}

	plan_execution::ExecutableMotionPlan plan;
	if (!constructMotionPlan(goal->solution, plan)) {
		// Abort here: falling through would let checkMoveitError() overwrite this with the
		// PREVIOUS execution's status and report a never-executed plan as SUCCESS.
		RCLCPP_ERROR(LOGGER, "Failed to construct motion plan from task solution - aborting goal");
		result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
		goal_handle->abort(result);
		return;
	}

	RCLCPP_INFO(LOGGER, "Executing TaskSolution");
	// Note: Planned path publishing is now handled in plan_execution.cpp
	// Each trajectory component's planned path is published when it starts executing
	result->error_code = context_->plan_execution_->executeAndMonitor(plan);

	// bounded wait with sleep instead of a hot spin: with the joinable monitor thread, executeAndMonitor()
	// already returns after execution finished, so this normally exits on the first iteration. The deadline
	// (> the 120 s monitor backstop) is a safety net so the action ALWAYS returns a result to the caller.
	const auto postcheck_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
	bool postcheck_timed_out = false;
	while(!context_->trajectory_execution_manager_->PostcheckExecCompleted()){ //depend on execution_complete_
		//wait until finish execution
		if (std::chrono::steady_clock::now() > postcheck_deadline) {
			postcheck_timed_out = true;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	if (postcheck_timed_out) {
		RCLCPP_ERROR(LOGGER, "Timed out waiting for trajectory execution to complete - aborting goal so the "
		                     "caller is not stuck waiting forever");
		context_->plan_execution_->stop();
		context_->trajectory_execution_manager_->stopExecution(true, false);
		result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
		goal_handle->abort(result);
		return;
	}
	// RCLCPP_INFO(LOGGER, "Finishing TaskSolution !!!!!!!!!!!!!!!!!!!!!!!!!");
	RCLCPP_DEBUG(LOGGER, "DEBUG: About to call checkMoveitError()");
	result->error_code = context_->plan_execution_->checkMoveitError();
	RCLCPP_DEBUG(LOGGER, "DEBUG: After checkMoveitError(), result->error_code.val = %d", result->error_code.val);
	if (result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
		RCLCPP_DEBUG(LOGGER, "DEBUG: Calling goal_handle->succeed() with error_code = %d", result->error_code.val);
		goal_handle->succeed(result);
	}
	else if (result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION){
		RCLCPP_DEBUG(LOGGER, "DEBUG: Calling goal_handle->succeed() (GOAL_IN_COLLISION) with error_code = %d", result->error_code.val);
		goal_handle->succeed(result);
	}
	else if (result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED && goal_handle->is_canceling()){
		RCLCPP_DEBUG(LOGGER, "DEBUG: Calling goal_handle->canceled() with error_code = %d", result->error_code.val);
		goal_handle->canceled(result);
	}
	else{
		RCLCPP_DEBUG(LOGGER, "DEBUG: Calling goal_handle->abort() with error_code = %d", result->error_code.val);
		goal_handle->abort(result);
	}

}

rclcpp_action::CancelResponse ExecuteTaskSolutionCapability::preemptCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTaskSolutionAction>>& /*goal_handle*/) {
	if (context_->plan_execution_)
		context_->plan_execution_->stop();
	return rclcpp_action::CancelResponse::ACCEPT;
}

bool ExecuteTaskSolutionCapability::constructMotionPlan(const moveit_task_constructor_msgs::msg::Solution& solution,
                                                        plan_execution::ExecutableMotionPlan& plan) {
	moveit::core::RobotModelConstPtr model = context_->planning_scene_monitor_->getRobotModel();

	moveit::core::RobotState state(model);
	{
		planning_scene_monitor::LockedPlanningSceneRO scene(context_->planning_scene_monitor_);
		state = scene->getCurrentState();
	}

	plan.plan_components_.reserve(solution.sub_trajectory.size());
	for (size_t i = 0; i < solution.sub_trajectory.size(); ++i) {
		const moveit_task_constructor_msgs::msg::SubTrajectory& sub_traj = solution.sub_trajectory[i];

		plan.plan_components_.emplace_back();
		plan_execution::ExecutableTrajectory& exec_traj = plan.plan_components_.back();

		// define individual variable for use in closure below
		const std::string description = std::to_string(i + 1) + "/" + std::to_string(solution.sub_trajectory.size());
		exec_traj.description_ = description;

		const moveit::core::JointModelGroup* group = nullptr;
		{
			std::vector<std::string> joint_names(sub_traj.trajectory.joint_trajectory.joint_names);
			joint_names.insert(joint_names.end(), sub_traj.trajectory.multi_dof_joint_trajectory.joint_names.begin(),
			                   sub_traj.trajectory.multi_dof_joint_trajectory.joint_names.end());
			if (!joint_names.empty()) {
				group = findJointModelGroup(*model, joint_names);
				if (!group) {
					RCLCPP_ERROR_STREAM(LOGGER, fmt::format("Could not find JointModelGroup that actuates {{{}}}",
					                                        fmt::join(joint_names, ", ")));
					return false;
				}
				RCLCPP_DEBUG(LOGGER, "Using JointModelGroup '%s' for execution", group->getName().c_str());
			}
		}
		exec_traj.trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(model, group);
		exec_traj.trajectory_->setRobotTrajectoryMsg(state, sub_traj.trajectory);

		// Check that sub trajectories that contain a valid trajectory have controllers configured.
		if (!sub_traj.trajectory.joint_trajectory.points.empty() && sub_traj.execution_info.controller_names.empty()) {
			RCLCPP_WARN(LOGGER,
			            "The trajectory of stage '%i' from task '%s' does not have any controllers specified for "
			            "trajectory execution. This might lead to unexpected controller selection.",
			            sub_traj.info.stage_id, solution.task_id.c_str());
		}
		exec_traj.controller_names_ = sub_traj.execution_info.controller_names;

		/* TODO add action feedback and markers */
		exec_traj.effect_on_success_ = [this,
		                                &scene_diff = const_cast<::moveit_msgs::msg::PlanningScene&>(sub_traj.scene_diff),
		                                description](const plan_execution::ExecutableMotionPlan* /*plan*/) {
			scene_diff.robot_state.joint_state = sensor_msgs::msg::JointState();
			scene_diff.robot_state.multi_dof_joint_state = sensor_msgs::msg::MultiDOFJointState();

			if (!moveit::core::isEmpty(scene_diff)) {
				RCLCPP_DEBUG_STREAM(LOGGER, "apply effect of " << description);
				return context_->planning_scene_monitor_->newPlanningSceneMessage(scene_diff);
			}
			return true;
		};

		if (!moveit::core::isEmpty(sub_traj.scene_diff.robot_state) &&
		    !moveit::core::robotStateMsgToRobotState(sub_traj.scene_diff.robot_state, state, true)) {
			RCLCPP_ERROR_STREAM(LOGGER, "invalid intermediate robot state in scene diff of SubTrajectory " << description);
			return false;
		}
	}

	return true;
}

}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(move_group::ExecuteTaskSolutionCapability, move_group::MoveGroupCapability)
