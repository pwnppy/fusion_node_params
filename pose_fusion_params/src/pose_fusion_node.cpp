#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>

class PoseTwistFusionNode : public rclcpp::Node {
public:
  PoseTwistFusionNode() : Node("pose_twist_fusion_node") {
    // Initialize parameters
    this->declare_parameter<std::string>("pose_topic", "/localization/pose_twist_fusion_filter/biased_pose_with_covariance_gnss");
    this->declare_parameter<std::string>("twist_topic", "/localization/pose_twist_fusion_filter/twist_with_covariance");
    this->declare_parameter<std::string>("lidar_pose_topic", "/localization/pose_twist_fusion_filter/biased_pose_with_covariance_lidar");
    this->declare_parameter<std::string>("lidar_twist_topic", "/localization/pose_twist_fusion_filter/twist_with_covariance");
    this->declare_parameter<std::string>("output_pose_topic", "/localization/pose_with_covariance");
    this->declare_parameter<std::string>("output_twist_topic", "/localization/pose_twist_fusion_filter/twist_with_covariance");
    this->declare_parameter<std::string>("output_kinematic_state_topic", "/localization/pose_twist_fusion_filter/kinematic_state");
    this->declare_parameter<std::string>("output_biased_pose_topic", "/localization/pose_twist_fusion_filter/biased_pose_with_covariance");
    this->declare_parameter<double>("gnss_pose_weight", 0.5);
    this->declare_parameter<double>("lidar_pose_weight", 0.5);
    this->declare_parameter<double>("gnss_twist_weight", 0.5);
    this->declare_parameter<double>("lidar_twist_weight", 0.5);

    // Read parameter values
    this->get_parameter("pose_topic", pose_topic_);
    this->get_parameter("twist_topic", twist_topic_);
    this->get_parameter("lidar_pose_topic", lidar_pose_topic_);
    this->get_parameter("lidar_twist_topic", lidar_twist_topic_);
    this->get_parameter("output_pose_topic", output_pose_topic_);
    this->get_parameter("output_twist_topic", output_twist_topic_);
    this->get_parameter("output_kinematic_state_topic", output_kinematic_state_topic_);
    this->get_parameter("output_biased_pose_topic", output_biased_pose_topic_);
    this->get_parameter("gnss_pose_weight", gnss_pose_weight_);
    this->get_parameter("lidar_pose_weight", lidar_pose_weight_);
    this->get_parameter("gnss_twist_weight", gnss_twist_weight_);
    this->get_parameter("lidar_twist_weight", lidar_twist_weight_);




    // Subscribers and Publishers
    sub_kinematic_state_lidar_ = this->create_subscription<nav_msgs::msg::Odometry>(
      lidar_pose_topic_, 10,
      std::bind(&PoseTwistFusionNode::callbackKinematicStateLidar, this, std::placeholders::_1));
    sub_kinematic_state_gnss_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, 10,
      std::bind(&PoseTwistFusionNode::callbackKinematicStateGnss, this, std::placeholders::_1));

    sub_biased_pose_lidar_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      lidar_twist_topic_, 10,
      std::bind(&PoseTwistFusionNode::callbackBiasedPoseLidar, this, std::placeholders::_1));
    sub_biased_pose_gnss_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      twist_topic_, 10,
      std::bind(&PoseTwistFusionNode::callbackBiasedPoseGnss, this, std::placeholders::_1));

    pub_fused_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_pose_topic_, 10);
    pub_fused_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      output_twist_topic_, 10);
    pub_fused_kinematic_state_ = this->create_publisher<nav_msgs::msg::Odometry>(
      output_kinematic_state_topic_, 10);
    pub_fused_biased_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_biased_pose_topic_, 10);
    pub_fused_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose", 10);


    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    max_buffer_size_ = 15;

    // Timer for fusion at 50hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),  // 50hz
      std::bind(&PoseTwistFusionNode::fusePoseAndTwist, this));
  }

private:
  // Member variables
  std::string pose_topic_;
  std::string twist_topic_;
  std::string lidar_pose_topic_;
  std::string lidar_twist_topic_;
  std::string output_pose_topic_;
  std::string output_twist_topic_;
  std::string output_kinematic_state_topic_;
  std::string output_biased_pose_topic_;
  double gnss_pose_weight_;
  double lidar_pose_weight_;
  double gnss_twist_weight_;
  double lidar_twist_weight_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> kinematic_state_lidar_buffer_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> kinematic_state_gnss_buffer_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> biased_pose_lidar_buffer_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> biased_pose_gnss_buffer_;

  size_t max_buffer_size_;

  // Fusion Timer Callback
  void fusePoseAndTwist() {
      nav_msgs::msg::Odometry fused_odom;
      fused_odom.header.stamp = this->now();
      fused_odom.header.frame_id = "map";

      double weight_lidar = 0.5;
      double weight_gnss = 0.5;

      auto latest_lidar_odom = !kinematic_state_lidar_buffer_.empty() ? kinematic_state_lidar_buffer_.back() : nullptr;
      auto latest_gnss_odom = !kinematic_state_gnss_buffer_.empty() ? kinematic_state_gnss_buffer_.back() : nullptr;

      if (!latest_lidar_odom && !latest_gnss_odom) {
          RCLCPP_WARN(this->get_logger(), "No LiDAR or GNSS data available for fusion.");
          return; 
      }

      if (latest_lidar_odom && !latest_gnss_odom) {
          fused_odom.pose.pose = latest_lidar_odom->pose.pose;
          fused_odom.twist.twist = latest_lidar_odom->twist.twist;

          RCLCPP_INFO(this->get_logger(), "Only LiDAR data used for fusion.");

      } else if (!latest_lidar_odom && latest_gnss_odom) {
          fused_odom.pose.pose = latest_gnss_odom->pose.pose;
          fused_odom.twist.twist = latest_gnss_odom->twist.twist;

          RCLCPP_INFO(this->get_logger(), "Only GNSS data used for fusion.");

      } else if (latest_lidar_odom && latest_gnss_odom) {
          double time_diff = std::abs((rclcpp::Time(latest_lidar_odom->header.stamp) - rclcpp::Time(latest_gnss_odom->header.stamp)).seconds());

          if (time_diff > 0.08) {
              RCLCPP_WARN(this->get_logger(), "Time difference between LiDAR and GNSS is too large: %f seconds", time_diff);

              if (rclcpp::Time(latest_lidar_odom->header.stamp) > rclcpp::Time(latest_gnss_odom->header.stamp)) {
                  fused_odom.pose.pose = latest_lidar_odom->pose.pose;
                  fused_odom.twist.twist = latest_lidar_odom->twist.twist;

                  RCLCPP_INFO(this->get_logger(), "Large time difference. Using latest LiDAR data.");
              } else {
                  fused_odom.pose.pose = latest_gnss_odom->pose.pose;
                  fused_odom.twist.twist = latest_gnss_odom->twist.twist;

                  RCLCPP_INFO(this->get_logger(), "Large time difference. Using latest GNSS data.");
              }
          } else {
              fused_odom.pose.pose.position.x = latest_lidar_odom->pose.pose.position.x * weight_lidar +
                                                latest_gnss_odom->pose.pose.position.x * weight_gnss;
              fused_odom.pose.pose.position.y = latest_lidar_odom->pose.pose.position.y * weight_lidar +
                                                latest_gnss_odom->pose.pose.position.y * weight_gnss;
              fused_odom.pose.pose.position.z = latest_lidar_odom->pose.pose.position.z * weight_lidar +
                                                latest_gnss_odom->pose.pose.position.z * weight_gnss;

              fused_odom.pose.pose.orientation = fuseOrientation(
                  latest_lidar_odom->pose.pose.orientation, latest_gnss_odom->pose.pose.orientation, weight_lidar, weight_gnss);

              fused_odom.twist.twist.linear.x = latest_lidar_odom->twist.twist.linear.x * weight_lidar +
                                                latest_gnss_odom->twist.twist.linear.x * weight_gnss;
              fused_odom.twist.twist.angular.z = latest_lidar_odom->twist.twist.angular.z * weight_lidar +
                                                latest_gnss_odom->twist.twist.angular.z * weight_gnss;

              RCLCPP_INFO(this->get_logger(), "LiDAR and GNSS data successfully fused.");
          }
      }

      pub_fused_kinematic_state_->publish(fused_odom);
      broadcastTransform(fused_odom.pose.pose);

      geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;
      fused_pose.header = fused_odom.header;
      fused_pose.pose = fused_odom.pose;

      geometry_msgs::msg::TwistWithCovarianceStamped fused_twist;
      fused_twist.header = fused_odom.header;
      fused_twist.twist = fused_odom.twist;

      pub_fused_pose_->publish(fused_pose);
      pub_fused_twist_->publish(fused_twist);

      geometry_msgs::msg::PoseStamped fused_pose_stamped;
      fused_pose_stamped.header = fused_odom.header;
      fused_pose_stamped.pose = fused_odom.pose.pose;
      
      pub_fused_pose_stamped_->publish(fused_pose_stamped);
  }

  // tf
  void broadcastTransform(const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = pose.position.x;
    transform_stamped.transform.translation.y = pose.position.y;
    transform_stamped.transform.translation.z = pose.position.z;
    transform_stamped.transform.rotation = pose.orientation;
    tf_br_->sendTransform(transform_stamped);
  }

  void callbackKinematicStateLidar(const nav_msgs::msg::Odometry::SharedPtr msg) {
    kinematic_state_lidar_buffer_.push_back(msg);
    if (kinematic_state_lidar_buffer_.size() > max_buffer_size_) {
      kinematic_state_lidar_buffer_.pop_front();
    }
  }

  void callbackKinematicStateGnss(const nav_msgs::msg::Odometry::SharedPtr msg) {
    kinematic_state_gnss_buffer_.push_back(msg);
    if (kinematic_state_gnss_buffer_.size() > max_buffer_size_) {
      kinematic_state_gnss_buffer_.pop_front();
    }
  }

  void callbackBiasedPoseLidar(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    biased_pose_lidar_buffer_.push_back(msg);
    if (biased_pose_lidar_buffer_.size() > max_buffer_size_) {
      biased_pose_lidar_buffer_.pop_front();
    }
    fuseBiasedPose();
  }

  void callbackBiasedPoseGnss(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    biased_pose_gnss_buffer_.push_back(msg);
    if (biased_pose_gnss_buffer_.size() > max_buffer_size_) {
      biased_pose_gnss_buffer_.pop_front();
    }
    fuseBiasedPose();
  }

  void fuseBiasedPose() {
    geometry_msgs::msg::PoseWithCovarianceStamped fused_biased_pose;
    fused_biased_pose.header.stamp = this->now();
    fused_biased_pose.header.frame_id = "map";

    double weight_lidar = 0.5;
    double weight_gnss = 0.5;

    if (!biased_pose_lidar_buffer_.empty() && biased_pose_gnss_buffer_.empty()) {
        auto latest_lidar_pose = biased_pose_lidar_buffer_.back();
        fused_biased_pose.pose.pose = latest_lidar_pose->pose.pose;

    } else if (biased_pose_lidar_buffer_.empty() && !biased_pose_gnss_buffer_.empty()) {
        auto latest_gnss_pose = biased_pose_gnss_buffer_.back();
        fused_biased_pose.pose.pose = latest_gnss_pose->pose.pose;

    } else if (!biased_pose_lidar_buffer_.empty() && !biased_pose_gnss_buffer_.empty()) {
        auto latest_lidar_pose = biased_pose_lidar_buffer_.back();
        auto closest_gnss_pose = findClosestPose(biased_pose_gnss_buffer_, latest_lidar_pose->header.stamp);
        if (!closest_gnss_pose) return;

        fused_biased_pose.pose.pose.position.x = latest_lidar_pose->pose.pose.position.x * weight_lidar +
                                                 closest_gnss_pose->pose.pose.position.x * weight_gnss;
        fused_biased_pose.pose.pose.position.y = latest_lidar_pose->pose.pose.position.y * weight_lidar +
                                                 closest_gnss_pose->pose.pose.position.y * weight_gnss;
        fused_biased_pose.pose.pose.position.z = latest_lidar_pose->pose.pose.position.z * weight_lidar +
                                                 closest_gnss_pose->pose.pose.position.z * weight_gnss;

        fused_biased_pose.pose.pose.orientation = fuseOrientation(
            latest_lidar_pose->pose.pose.orientation, closest_gnss_pose->pose.pose.orientation, weight_lidar, weight_gnss);
    }

    pub_fused_biased_pose_->publish(fused_biased_pose);
  }

  geometry_msgs::msg::Quaternion fuseOrientation(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2, double w1, double w2) {
    tf2::Quaternion quat1, quat2;
    tf2::fromMsg(q1, quat1);
    tf2::fromMsg(q2, quat2);
    tf2::Quaternion fused_quat = quat1 * w1 + quat2 * w2;
    fused_quat.normalize();
    return tf2::toMsg(fused_quat);
  }

  std::array<double, 36> fuseCovariance(const std::array<double, 36>& cov1, const std::array<double, 36>& cov2, double w1, double w2) {
    std::array<double, 36> fused_cov;
    for (size_t i = 0; i < 36; ++i) {
      fused_cov[i] = cov1[i] * w1 + cov2[i] * w2;
    }
    return fused_cov;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr findClosestPose(
    const std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>& buffer,
    const builtin_interfaces::msg::Time& target_time) {
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr closest_msg = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();

    rclcpp::Time target_time_rclcpp(target_time);
    for (const auto& msg : buffer) {
      rclcpp::Time msg_time(msg->header.stamp);
      double time_diff = std::abs((msg_time - target_time_rclcpp).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_msg = msg;
      }
    }
    return closest_msg;
  }

  nav_msgs::msg::Odometry::SharedPtr findClosestOdometry(
    const std::deque<nav_msgs::msg::Odometry::SharedPtr>& buffer,
    const builtin_interfaces::msg::Time& target_time) {
    nav_msgs::msg::Odometry::SharedPtr closest_msg = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();

    rclcpp::Time target_time_rclcpp(target_time);
    for (const auto& msg : buffer) {
      rclcpp::Time msg_time(msg->header.stamp);
      double time_diff = std::abs((msg_time - target_time_rclcpp).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_msg = msg;
      }
    }
    return closest_msg;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_lidar_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_gnss_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_biased_pose_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_biased_pose_gnss_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_fused_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_fused_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_fused_kinematic_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_fused_biased_pose_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_fused_pose_stamped_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTwistFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}