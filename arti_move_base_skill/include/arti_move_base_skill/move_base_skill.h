/*
Created by clemens on 28.04.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_SKILL_MOVE_BASE_SKILL_H
#define ARTI_MOVE_BASE_SKILL_MOVE_BASE_SKILL_H

#include <arti_movement_skill_interface/abstract_movement_skill.h>
#include <actionlib/client/simple_action_client.h>
#include <arti_move_base_msgs/MoveInNetworkAction.h>
#include <boost/optional.hpp>

namespace arti_move_base_skill
{

class MoveBaseSkill : public arti_movement_skill_interface::AbstractMovementSkill
{
public:
  void initialize(
    std::string name, const ros::NodeHandle& node_handle, ResultCB finished_with_success_cb,
    ResultCB finished_with_failure_cb) override;

  double estimateCosts(
    const arti_graph_processing::VertexPtr& source, const arti_graph_processing::VertexPtr& destination) override;

  void performSkill(
    const arti_graph_processing::VertexPtr& source, const arti_graph_processing::VertexPtr& destination) override;

  void stopPerformingSkill() override;

private:
  using ActionClient = actionlib::SimpleActionClient<arti_move_base_msgs::MoveInNetworkAction>;

  void doneCB(
    const actionlib::SimpleClientGoalState& state, const arti_move_base_msgs::MoveInNetworkResultConstPtr& result);

  boost::optional<ActionClient> action_client_;
};

}

#endif //ARTI_MOVE_BASE_SKILL_MOVE_BASE_SKILL_H
