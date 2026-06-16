#include <rclcpp/rclcpp.hpp>
#include <manipulation_controller_interfaces/srv/follow_trajectory.hpp>
#include <arms_interfaces/srv/move_arms.hpp>
#include <torso_interfaces/srv/move_torso.hpp>
#include <vector>
#include <cmath> 

using namespace std::chrono_literals;

class ManipulationControllerNode : public rclcpp::Node
{
public:
    ManipulationControllerNode() : Node("manipulation_controller_node")
    {
        client_node_ = rclcpp::Node::make_shared("manipulation_controller_client_node");

        right_arm_client_ = client_node_->create_client<arms_interfaces::srv::MoveArms>("/hardware/arms/move_right_arm");
        left_arm_client_  = client_node_->create_client<arms_interfaces::srv::MoveArms>("/hardware/arms/move_left_arm");
        torso_client_     = client_node_->create_client<torso_interfaces::srv::MoveTorso>("/hardware/torso/move_torso");

        cb_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        cb_executor_->add_node(client_node_);

        follow_trajectory_srv_ = this->create_service<manipulation_controller_interfaces::srv::FollowTrajectory>(
            "follow_trajectory",
            std::bind(&ManipulationControllerNode::followTrajectoryCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Manipulation controller ready.");
    }

private:
    rclcpp::Node::SharedPtr                                                              client_node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr                                 cb_executor_;
    rclcpp::Service<manipulation_controller_interfaces::srv::FollowTrajectory>::SharedPtr follow_trajectory_srv_;
    rclcpp::Client<arms_interfaces::srv::MoveArms>::SharedPtr                            right_arm_client_;
    rclcpp::Client<arms_interfaces::srv::MoveArms>::SharedPtr                            left_arm_client_;
    rclcpp::Client<torso_interfaces::srv::MoveTorso>::SharedPtr                          torso_client_;

	const int SPEED_MAX = 50;

    void followTrajectoryCallback(
        const std::shared_ptr<manipulation_controller_interfaces::srv::FollowTrajectory::Request> request,
        std::shared_ptr<manipulation_controller_interfaces::srv::FollowTrajectory::Response> response)
    {
        auto &arm_client_ = (request->planning_group == "right_arm_group") ? right_arm_client_ : left_arm_client_;

        const size_t DOF_TORSO  = 1;
        const size_t DOF_ARM    = 6;
        const size_t BLOCK_SIZE = DOF_TORSO + DOF_ARM;

        if (request->waypoints.size() % BLOCK_SIZE != 0) {
            RCLCPP_ERROR(this->get_logger(), "Waypoints array size (%zu) is not a multiple of %zu",
                         request->waypoints.size(), BLOCK_SIZE);
            response->success = false;
            response->message = "Invalid waypoints array size";
            return;
        }

        size_t num_blocks = request->waypoints.size() / BLOCK_SIZE;
        RCLCPP_INFO(this->get_logger(), "Executing trajectory with %zu blocks", num_blocks);

		if (num_blocks == 0)
		{
			response->success = true;
			response->message = "Empty trajectory";
			return;
		}

		if (!torso_client_->wait_for_service(5s))
		{
			response->success = false;
			response->message = "Torso service unavailable";
			return;
		}

		if (!arm_client_->wait_for_service(5s))
		{
			response->success = false;
			response->message = "Arm service unavailable";
			return;
		}

		for (size_t block = 0; block < num_blocks; block++)
        {
            double torso_position = request->waypoints[block * BLOCK_SIZE];
            std::vector<double> arm_positions(
                request->waypoints.begin() + block * BLOCK_SIZE + 1,
                request->waypoints.begin() + (block + 1) * BLOCK_SIZE);

            auto torso_req = std::make_shared<torso_interfaces::srv::MoveTorso::Request>();
            torso_req->position = torso_position;

			auto arm_req = std::make_shared<arms_interfaces::srv::MoveArms::Request>();
			arm_req->positions = arm_positions;
			arm_req->speed = SPEED_MAX;

            auto torso_future = torso_client_->async_send_request(torso_req);
            auto arm_future   = arm_client_->async_send_request(arm_req);

            auto torso_status = cb_executor_->spin_until_future_complete(torso_future, 5s);
            auto arm_status   = cb_executor_->spin_until_future_complete(arm_future,   5s);

            if (torso_status == rclcpp::FutureReturnCode::SUCCESS) {
                auto result = torso_future.get();
                if (result->success)
                    RCLCPP_INFO(this->get_logger(), "[Block %zu] Torso OK: %s", block, result->message.c_str());
                else
                    RCLCPP_WARN(this->get_logger(), "[Block %zu] Torso failed: %s", block, result->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "[Block %zu] Torso timed out", block);
            }

            if (arm_status == rclcpp::FutureReturnCode::SUCCESS) {
                auto result = arm_future.get();
                if (result->success)
                    RCLCPP_INFO(this->get_logger(), "[Block %zu] Arm OK: %s", block, result->message.c_str());
                else
                    RCLCPP_WARN(this->get_logger(), "[Block %zu] Arm failed: %s", block, result->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "[Block %zu] Arm timed out", block);
            }
        }

        response->success = true;
        response->message = "Trajectory executed";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulationControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
