/*
Created by clemens on 28.04.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_move_base_skill/move_base_skill.h>
#include <arti_graph_processing/vertex.h>
#include <pluginlib/class_list_macros.h>
#include <stdexcept>
#include <tf/transform_datatypes.h>

namespace arti_move_base_skill
{

void MoveBaseSkill::initialize(
  std::string name, const ros::NodeHandle& node_handle, ResultCB finished_with_success_cb,
  ResultCB finished_with_failure_cb)
{
  AbstractMovementSkill::initialize(name, node_handle, finished_with_success_cb, finished_with_failure_cb);

  action_client_.emplace(node_handle_, "/move_in_network", true);

  while (!action_client_->waitForServer(ros::Duration(2.)) && ros::ok())
  {
    ROS_WARN("arti_move_base move_in_network action server not yet up");
  }
}

double MoveBaseSkill::estimateCosts(
  const arti_graph_processing::VertexPtr& source, const arti_graph_processing::VertexPtr& destination)
{
  return source->calculateEuclideanDistanceTo(*destination);
}

void MoveBaseSkill::performSkill(
  const arti_graph_processing::VertexPtr& /*source*/, const arti_graph_processing::VertexPtr& destination)
{
  if (!action_client_)
  {
    throw std::logic_error("MoveBaseSkill: called performSkill before initialize");
  }

  if (!action_client_->getState().isDone())
  {
    ROS_WARN_STREAM("MoveBaseSkill: called performSkill while another goal is still active");
    // Not cancelling goal here, as that might (?) result in a done callback.
  }

  arti_move_base_msgs::MoveInNetworkGoal goal;
  goal.target_pose.pose.point.x.value = destination->getPose().pose.position.x;
  goal.target_pose.pose.point.y.value = destination->getPose().pose.position.y;
  goal.target_pose.pose.theta.value = tf::getYaw(destination->getPose().pose.orientation);

  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = destination->getPose().header.frame_id;

  action_client_->sendGoal(goal, boost::bind(&MoveBaseSkill::doneCB, this, _1, _2));
}

void MoveBaseSkill::stopPerformingSkill()
{
  if (!action_client_)
  {
    ROS_ERROR_STREAM("MoveBaseSkill: called stopPerformingSkill before initialize");
  }

  if (action_client_->getState().isDone())
  {
    ROS_WARN_STREAM("MoveBaseSkill: called stopPerformingSkill while no goal is active");
  }

  action_client_->cancelAllGoals();
}

void MoveBaseSkill::doneCB(
  const actionlib::SimpleClientGoalState& state, const arti_move_base_msgs::MoveInNetworkResultConstPtr& /*result*/)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    if (finished_with_success_cb_)
    {
      finished_with_success_cb_();
    }
  }
  else if (state != actionlib::SimpleClientGoalState::ACTIVE)
  {
    if (finished_with_failure_cb_)
    {
      finished_with_failure_cb_();
    }
  }
}
}

PLUGINLIB_EXPORT_CLASS(arti_move_base_skill::MoveBaseSkill, arti_movement_skill_interface::MovementSkillInterface)