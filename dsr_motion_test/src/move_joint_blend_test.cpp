#include "rclcpp/rclcpp.hpp"
#include "dsr_msgs2/srv/move_joint.hpp"
#include <chrono>
#include <array>
#include <vector>
#include <memory>

using namespace std::chrono_literals;

class MoveJointBlendTest : public rclcpp::Node
{
public:
  MoveJointBlendTest()
  : Node("move_joint_blend_test")
  {
    // 파라미터 설정
    this->declare_parameter("robot_name", "dsr01");
    this->declare_parameter("velocity", 100.0);
    this->declare_parameter("acceleration", 10.0);
    this->declare_parameter("radius", 5.0);
    this->declare_parameter("sync_type", 1);  // 0=SYNC, 1=ASYNC
    this->declare_parameter("blend_type", 0); // 기본값 0
    this->declare_parameter("mode", 0);       // 절대모드(ABSOLUTE)

    // 서비스 이름 구성
    std::string robot_name = this->get_parameter("robot_name").as_string();
    std::string service_name = "/" + robot_name + "/motion/move_joint";

    RCLCPP_INFO(this->get_logger(), "Connecting to service: %s", service_name.c_str());
    client_ = this->create_client<dsr_msgs2::srv::MoveJoint>(service_name);

    // 테스트용 joint trajectory (degree)
    test_points_ = {
      {0.0, 0.0, 90.0, 0.0, 90.0, 0.0},
      {10.0, 5.0, 90.0, 0.0, 90.0, 0.0},
      {30.0, 5.0, 90.0, 0.0, 90.0, 0.0},
      {50.0, 5.0, 90.0, 0.0, 90.0, 0.0},
      {70.0, 5.0, 90.0, 0.0, 90.0, 0.0},
      {90.0, 5.0, 90.0, 0.0, 90.0, 0.0}
    };

    // 2초 후 테스트 실행
    timer_ = this->create_wall_timer(2s, std::bind(&MoveJointBlendTest::execute_trajectory, this));
  }

private:
  void execute_trajectory()
  {
    timer_->cancel();

    if (!client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "MoveJoint service not available!");
      return;
    }

    double vel = this->get_parameter("velocity").as_double();
    double acc = this->get_parameter("acceleration").as_double();
    double rad = this->get_parameter("radius").as_double();
    int sync = this->get_parameter("sync_type").as_int();
    int blend_type = this->get_parameter("blend_type").as_int();
    int mode = this->get_parameter("mode").as_int();

    RCLCPP_INFO(this->get_logger(),
                "Executing test: vel=%.1f, acc=%.1f, rad=%.1f, sync=%d",
                vel, acc, rad, sync);

    for (size_t i = 0; i < test_points_.size(); ++i)
    {
      auto req = std::make_shared<dsr_msgs2::srv::MoveJoint::Request>();

      // pos: std::array<double,6>
      for (size_t j = 0; j < 6; ++j)
        req->pos[j] = test_points_[i][j];

      req->vel = vel;
      req->acc = acc;
      req->time = 0.0;
      req->radius = rad;
      req->mode = mode;
      req->blend_type = blend_type;
      req->sync_type = sync;

      RCLCPP_INFO(this->get_logger(), "[%zu/%zu] Sending MoveJoint...", i + 1, test_points_.size());
      auto future = client_->async_send_request(req);

      if (sync == 0) {
        if (future.wait_for(10s) == std::future_status::timeout) {
          RCLCPP_WARN(this->get_logger(), "Timeout waiting for response");
          continue;
        }
        auto res = future.get();
        if (!res->success)
          RCLCPP_ERROR(this->get_logger(), "MoveJoint failed");
        else
          RCLCPP_INFO(this->get_logger(), "MoveJoint success");
      } else {
        rclcpp::sleep_for(100ms);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory test finished");
  }

  rclcpp::Client<dsr_msgs2::srv::MoveJoint>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::array<double, 6>> test_points_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveJointBlendTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
