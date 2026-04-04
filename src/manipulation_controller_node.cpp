#include <rclcpp/rclcpp.hpp>
#include <manipulation_controller_interfaces/srv/follow_trajectory.hpp>
#include <arms_interfaces/srv/move_arms.hpp>
#include <torso_interfaces/srv/move_torso.hpp>
#include <vector>

using namespace std::chrono_literals;

class ManipulationControllerNode : public rclcpp::Node
{
public:
    ManipulationControllerNode() : Node("manipulation_controller_node")
    {
        follow_trajectory_srv_ = this->create_service<manipulation_controller_interfaces::srv::FollowTrajectory>(
            "follow_trajectory",
            std::bind(&ManipulationControllerNode::followTrajectoryCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        right_arm_client_ = this->create_client<arms_interfaces::srv::MoveArms>("/hardware/arms/move_right_arm");
        left_arm_client_  = this->create_client<arms_interfaces::srv::MoveArms>("/hardware/arms/move_left_arm");
        torso_client_     = this->create_client<torso_interfaces::srv::MoveTorso>("/hardware/torso/move_torso");

        RCLCPP_INFO(this->get_logger(), "Manipulation controller ready.");
    }

private:
    rclcpp::Service<manipulation_controller_interfaces::srv::FollowTrajectory>::SharedPtr follow_trajectory_srv_;
    rclcpp::Client<arms_interfaces::srv::MoveArms>::SharedPtr right_arm_client_;
    rclcpp::Client<arms_interfaces::srv::MoveArms>::SharedPtr left_arm_client_;
    rclcpp::Client<torso_interfaces::srv::MoveTorso>::SharedPtr torso_client_;

    void followTrajectoryCallback(
        const std::shared_ptr<manipulation_controller_interfaces::srv::FollowTrajectory::Request> request,
        std::shared_ptr<manipulation_controller_interfaces::srv::FollowTrajectory::Response> response)
    {
        auto &arm_client = (request->planning_group == "right_arm_group") ? right_arm_client_ : left_arm_client_;

		const size_t DOF_TORSO = 1;
        const size_t DOF_ARM = 6;
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

        for (size_t block = 0; block < num_blocks; block++)
        {
            double torso_position = request->waypoints[block * BLOCK_SIZE];
            std::vector<double> arm_positions(
                request->waypoints.begin() + block * BLOCK_SIZE + 1,
                request->waypoints.begin() + (block + 1) * BLOCK_SIZE);

            // --- Torso request ---
            auto torso_req = std::make_shared<torso_interfaces::srv::MoveTorso::Request>();
            torso_req->position = torso_position;

            if (!torso_client_->wait_for_service(1s)) {
                RCLCPP_ERROR(this->get_logger(), "[Block %zu] Torso service unavailable", block);
                continue;
            }

            torso_client_->async_send_request(torso_req,
                [block, this](rclcpp::Client<torso_interfaces::srv::MoveTorso>::SharedFuture future){
                    auto result = future.get();
                    if (result->success)
                        RCLCPP_INFO(this->get_logger(), "[Block %zu] Torso movement succeeded: %s", block, result->message.c_str());
                    else
                        RCLCPP_ERROR(this->get_logger(), "[Block %zu] Torso movement failed: %s", block, result->message.c_str());
                });

            // --- Arm request ---
            auto arm_req = std::make_shared<arms_interfaces::srv::MoveArms::Request>();
            arm_req->positions = arm_positions;

            if (!arm_client->wait_for_service(1s)) {
                RCLCPP_ERROR(this->get_logger(), "[Block %zu] Arm service unavailable", block);
                continue;
            }

            arm_client->async_send_request(arm_req,
                [block, this](rclcpp::Client<arms_interfaces::srv::MoveArms>::SharedFuture future){
                    auto result = future.get();
                    if (result->success)
                        RCLCPP_INFO(this->get_logger(), "[Block %zu] Arm movement succeeded: %s", block, result->message.c_str());
                    else
                        RCLCPP_ERROR(this->get_logger(), "[Block %zu] Arm movement failed: %s", block, result->message.c_str());
                });

            std::this_thread::sleep_for(500ms);
        }

        response->success = true;
        response->message = "Trajectory requests sent";
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
