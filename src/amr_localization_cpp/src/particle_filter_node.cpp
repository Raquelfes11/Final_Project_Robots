// ROS 2 libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <cmath>

#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "amr_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

double degrees_to_radians(double degrees) {
    return degrees * (M_PI / 180.0);
}

class ParticleFilterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// ParticleFilterNode constructor
    ParticleFilterNode() : LifecycleNode{"particle_filter"}
    {
        this->declare_parameter("dt", 0.05);
        this->declare_parameter("enable_plot", false);
        this->declare_parameter("global_localization", true);
        this->declare_parameter("initial_pose", std::make_tuple(0.0, 0.0, degrees_to_radians(0)));  // (x, y, theta)
        this->declare_parameter("initial_pose_sigma", std::make_tuple(0.05, 0.05, 0.1));  // (sigma_x, sigma_y, sigma_theta)
        this->declare_parameter("particles", 1000);
        this->declare_parameter("sigma_v", 0.1);
        this->declare_parameter("sigma_w", 0.1);
        this->declare_parameter("sigma_z", 0.1);
        this->declare_parameter("steps_btw_sense_updates", 10);
        this->declare_parameter("world", "project");
    }

    /**
     * Callback called when the lifecycle node enters the "configuring" state.
     * Depending on the return value of this function, the state machine
     * either invokes a transition to the "inactive" state or stays in "unconfigured".
     * TRANSITION_CALLBACK_SUCCESS transitions to "inactive".
     * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured".
     * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing".
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Attempting a 'configure' transition from the '%s' state.",
                    state.label().c_str());

        
        // Parameters
        double dt;
        bool enable_plot;
        bool global_localization;
        std::tuple<double, double, double> initial_pose;
        std::tuple<double, double, double> initial_pose_sigma;
        int particles;
        double sigma_v, sigma_w, sigma_z;
        int steps_btw_sense_updates;
        std::string world;

        this->get_parameter("dt", dt);
        this->get_parameter("enable_plot", enable_plot);
        this->get_parameter("global_localization", global_localization);
        this->get_parameter("initial_pose", initial_pose);
        this->get_parameter("initial_pose_sigma", initial_pose_sigma);
        this->get_parameter("particles", particles);
        this->get_parameter("sigma_v", sigma_v);
        this->get_parameter("sigma_w", sigma_w);
        this->get_parameter("sigma_z", sigma_z);
        this->get_parameter("steps_btw_sense_updates", steps_btw_sense_updates);
        this->get_parameter("world", world);

        // Attribute and object initializations
        _localized = false;
        _steps = 0;

        std::filesystem::path map_path = std::filesystem::path(__FILE__).parent_path().parent_path() / "maps" / (world + ".json");

        // Particle Filter
        _particle_filter = std::make_shared<ParticleFilter>(
            dt, map_path.string(), particles, sigma_v, sigma_w, sigma_z, global_localization,
            initial_pose, initial_pose_sigma
        );

        // Publisher
        publisher_pose_ = this->create_publisher<amr_msgs::msg::PoseStamped>("/pose", 10);

        // Suscribers
        rclcpp::QoS qos_profile{
            rclcpp::QoSInitialization{RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10},
            rmw_qos_profile_sensor_data};


        odom_sub.subscribe(this, "/odometry", 10);
        scan_sub.subscribe(this, "/scan", 10);

        _sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>>(message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,sensor_msgs::msg::LaserScan>(10), odom_sub, scan_sub);
        sync->setAgePenalty(9.0);
        sync->registerCallback(std::bind(&ParticleFilterNode::_compute_pose_callback, this, _1, _2));

        return LifecycleNode::on_configure(state);
    }

    /**
     * Callback called when the lifecycle node enters the "activating" state.
     * Depending on the return value of this function, the state machine
     * either invokes a transition to the "active" state or stays in "inactive".
     * TRANSITION_CALLBACK_SUCCESS transitions to "active".
     * TRANSITION_CALLBACK_FAILURE transitions to "inactive".
     * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing".
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Attempting an 'activate' transition from the '%s' state.",
                    state.label().c_str());

        return LifecycleNode::on_activate(state);
    }

private:
    // Callbacks
    /**
     * Subscriber callback. Executes a particle filter and publishes (x, y, theta) estimates.

     * Args:
         * odom_msg: Message containing odometry measurements.
         * scan_msg: Message containing LiDAR sensor readings.

     */
    void _compute_pose_callback(
        const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
        const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        float z_v = odom_msg.twist.twist.linear.x;
        float z_w = odom_msg.twist.twist.angular.z;
        const std::vector<float> z_scan = scan_msg.ranges;

        _execute_motion_step(z_v, z_w);

        double x_h, y_h, theta_h;
        std::tie(x_h, y_h, theta_h) = _execute_measurement_step(z_scan);
        _steps += 1;

        // Publish
        _publish_pose_estimate(x_h, y_h, theta_h);
    }

    /**
     * Executes and monitors the motion step (move) of the particle filter.

     * Args:
         * z_v: Odometric estimate of the linear velocity of the robot center [m/s].
         * z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
     */
    void _execute_motion_step(float z_v, float z_w)
    {
        auto start = std::chrono::high_resolution_clock::now();

        _particle_filter->move(z_v, z_w);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        RCLCPP_INFO(this->get_logger(), "Move step time: %7.3f s", elapsed.count());
    }

    /**
     * Executes and monitors the measurement step (sense) of the particle filter.

     * Args:
         * z_us: Distance from every ultrasonic sensor to the closest obstacle [m].
        
     * Returns:
         * Pose estimate (x_h, y_h, theta_h) [m, m, rad]; inf if cannot be computed.

     */
    std::tuple<double, double, double> _execute_measurement_step(const std::vector<float> &z_us)
    {
        std::tuple<double, double, double> pose{std::numeric_limits<double>::infinity(),
                                                std::numeric_limits<double>::infinity(),
                                                std::numeric_limits<double>::infinity()};

        if (_localized || (_steps % _steps_btw_sense_updates == 0))
        {
            auto start_time = std::chrono::high_resolution_clock::now();
    
            _particle_filter->resample(z_us);
    
            auto end_time = std::chrono::high_resolution_clock::now();
            auto sense_time = std::chrono::duration<double>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), "Sense step time: %6.3f s", sense_time);
    
            start_time = std::chrono::high_resolution_clock::now();
    
            std::tie(_localized, pose) = _particle_filter->compute_pose();
    
            end_time = std::chrono::high_resolution_clock::now();
            auto clustering_time = std::chrono::duration<double>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), "Clustering time: %6.3f s", clustering_time);
        }
                                            
        return pose;
    }

    /**
     * Publishes the robot's pose estimate in a custom amr_msgs.msg.PoseStamped message.

     * Args:
         * x_h: x coordinate estimate [m].
         * y_h: y coordinate estimate [m].
         * theta_h: Heading estimate [rad].
     */
    void _publish_pose_estimate(double x_h, double y_h, double theta_h)
    {
        amr_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.localized = _localized;

        if (_localized)
        {
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, theta_h);

            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();
            pose_msg.pose.position.x = x_h;
            pose_msg.pose.position.y = y_h;
        }

        publisher_pose_->publish(pose_msg);
    }

    // Attributes
    std::shared_ptr<ParticleFilter> _particle_filter;
    bool _localized;
    int _steps;

    rclcpp::Publisher<amr_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;

    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>>> _sync;

};


int main(int argc, char *argv[])
{
    // Force flush the stdout buffer.
    // This ensures a correct synchronization of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    std::shared_ptr<ParticleFilterNode> node = std::make_shared<ParticleFilterNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}