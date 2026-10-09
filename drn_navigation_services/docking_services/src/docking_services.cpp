#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "drn_perception_msgs/srv/docks_detected.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <drn_navigation_msgs/action/docking_param.hpp>


using namespace std::placeholders;

class DockNavigationServer : public rclcpp::Node
{
public:
  DockNavigationServer()
  : Node("dock_navigation_server")
  { 
    RCLCPP_INFO(this->get_logger(), "Init DockingService...");
    
    odom_cb_group =  this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // action_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    odom_options.callback_group = odom_cb_group;    
    // client_options.callback_group = client_cb_group;
    // action_options.callback_group = action_cb_group;
    
    tf_buffer_1 = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_1 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1);
    
    // Init clients
    dock_detection_client_ = this->create_client<drn_perception_msgs::srv::DocksDetected>("/drn_perception/dock_detector/dock_detection",rmw_qos_profile_services_default,client_cb_group);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/mrac_controller/command", 10);
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&DockNavigationServer::odom_callback, this, std::placeholders::_1),odom_options);

    // Iniciar o servidor de serviço

    
    action_server_ = rclcpp_action::create_server<drn_navigation_msgs::action::DockingParam>(
        this,
        "docking_align",
        std::bind(&DockNavigationServer::handle_goal, this, _1,_2),  
        std::bind(&DockNavigationServer::handle_cancel, this,_1),     
        std::bind(&DockNavigationServer::handle_execute, this,_1)
    );
    

    RCLCPP_INFO(this->get_logger(), "Action '/docking_align' init.");
  }

private:
    rclcpp::SubscriptionOptions odom_options;
    // rclcpp::SubscriptionOptions client_options;
    // rclcpp::SubscriptionOptions action_options;
    rclcpp::CallbackGroup::SharedPtr odom_cb_group;
    rclcpp::CallbackGroup::SharedPtr client_cb_group;
    // rclcpp::CallbackGroup::SharedPtr action_cb_group;
    rclcpp::Client<drn_perception_msgs::srv::DocksDetected>::SharedPtr dock_detection_client_;
    rclcpp_action::Server<drn_navigation_msgs::action::DockingParam>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_1;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_1;
    int dock_index = 0;
    bool obstacle_detected_ = false;
    bool start_movement_ = false;
    bool first_time = false;
    bool move_forward_ = false;
    bool finish_action = false;
    bool not_dock = false;
    geometry_msgs::msg::PoseStamped start_position_;
    geometry_msgs::msg::PoseStamped last_position_;
    double target_angle;
    double distance_to_target;
    double Kp_linear;
    double Kp_angular;
    double linear_speed;
    double angular_speed;

    double distance_seg;
    double linear_tolerance;
    double angular_tolerance;
    std::string direction;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const drn_navigation_msgs::action::DockingParam::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order: distan_seg (%f) e linear_tolerance (%f) ", goal->distance_seg, goal->linear_tolerance);
        
        if (!dock_detection_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Serviço de detecção de dock não disponível.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        auto request_detection = std::make_shared<drn_perception_msgs::srv::DocksDetected::Request>();
        request_detection->direction = goal->direction;
        request_detection->base_frame = "base_link";
        direction = goal->direction;
        distance_seg = goal->distance_seg;
        linear_tolerance = goal->linear_tolerance;
        angular_tolerance = goal->angular_tolerance;
        finish_action = false;
        not_dock = false;
        

        auto future = dock_detection_client_->async_send_request(
            request_detection,
            std::bind(&DockNavigationServer::dock_detection_callback, this, std::placeholders::_1));
        
        if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout ao aguardar resposta do serviço.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // auto response = future.get();
        // if (!response->dock_detected || response->docks.empty())
        // {
        //     RCLCPP_WARN(this->get_logger(), "Nenhum dock detectado. Rejeitando a solicitação de docagem.");
        //     return rclcpp_action::GoalResponse::REJECT;
        // }

        if(direction != "front" && direction != "rear"){
            RCLCPP_WARN(this->get_logger(), "Direction not accepted.");
            return rclcpp_action::GoalResponse::REJECT;
        } 
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<drn_navigation_msgs::action::DockingParam>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(twist_msg);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<drn_navigation_msgs::action::DockingParam>> goal_handle)
    {
        auto feedback = std::make_shared<drn_navigation_msgs::action::DockingParam::Feedback>();
        auto result = std::make_shared<drn_navigation_msgs::action::DockingParam::Result>();
        auto twist_msg = geometry_msgs::msg::Twist();
        
        if(not_dock == true){
            result->success = false;
            goal_handle->succeed(result);
            return;
        }
        while(rclcpp::ok() && !finish_action){
            double error_angle;
            double error;

            if (goal_handle->is_canceling() && !rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "O objetivo foi cancelado");
                goal_handle->canceled(result);
                return;
            }

            // RCLCPP_INFO(this->get_logger(), "Entrei e start_movent é %d",start_movement_);
            result->success = false;

            if(start_movement_){
        
                tf2::Quaternion q;
                tf2::fromMsg(last_position_.pose.orientation, q);
                double roll_last, pitch_last, yaw_last;
                tf2::Matrix3x3(q).getRPY(roll_last, pitch_last, yaw_last);
                // RCLCPP_INFO(this->get_logger(), "Yaw_last: %.2f rad (%.2f deg)", yaw_last, yaw_last * 180.0 / M_PI);

                tf2::Quaternion q2;
                tf2::fromMsg(start_position_.pose.orientation, q2);
                double roll_start, pitch_start, yaw_start;
                tf2::Matrix3x3(q2).getRPY(roll_start, pitch_start, yaw_start);
                // RCLCPP_INFO(this->get_logger(), "Yaw_start: %.2f rad (%.2f deg)", yaw_start, yaw_start * 180.0 / M_PI);

                if(std::abs(yaw_last)>(M_PI/2) && yaw_last < 0){
                    yaw_last = yaw_last + 2*M_PI;
                }

                if(std::abs(yaw_start)>(M_PI/2) && yaw_start < 0){
                    yaw_start = yaw_start + 2*M_PI;
                }

                double angle_traveled = yaw_last - yaw_start;
                error_angle = target_angle - angle_traveled;
                // RCLCPP_INFO(this->get_logger(), "Erro angular: %.2f rad", error_angle);
                
                const double max_angular_speed = 0.1;  // Velocidade angular máxima (rad/s)

                double angular_speed = Kp_angular * error_angle;
                
                // angular_speed = std::clamp(angular_speed, -max_angular_speed, max_angular_speed);
                if(angular_speed < -max_angular_speed){
                    angular_speed = -max_angular_speed;
                }

                if(angular_speed > max_angular_speed){
                    angular_speed = max_angular_speed;
                }
                // RCLCPP_INFO(this->get_logger(), "Velocidade angular: %.2f rad", angular_speed);
                
                twist_msg.angular.z = angular_speed;
                cmd_vel_publisher_->publish(twist_msg);
                if (std::abs(error_angle) < angular_tolerance)  // Tolerância de 0.1 rad
                {
                    twist_msg.angular.z = 0.0;
                    cmd_vel_publisher_->publish(twist_msg);
                    start_movement_ = false;
                    move_forward_ = true;
                    RCLCPP_INFO(this->get_logger(), "Rotação finalizada!");
                }
            }
            else{
                if(!start_movement_ && move_forward_){
                    // RCLCPP_INFO(this->get_logger(), "Distância: %.2f",distance_to_target);
                    double dx = last_position_.pose.position.x - start_position_.pose.position.x;
                    double dy = last_position_.pose.position.y - start_position_.pose.position.y;
                    double distance_traveled_ = std::sqrt(dx * dx + dy * dy);
                    
                    if(distance_to_target < 0){
                        error = distance_to_target + distance_traveled_;
                        // RCLCPP_INFO(this->get_logger(), "Entrei + distancia target: %.2f m e distancia_traveled : %.2f", distance_to_target,distance_traveled_);

                    }else{
                        error = distance_to_target - distance_traveled_;
                        // RCLCPP_INFO(this->get_logger(), "Entrei - distancia target: %.2f m e distancia_traveled : %.2f", distance_to_target,distance_traveled_);
                    }
                    
                    // RCLCPP_INFO(this->get_logger(), "distancia traveled: %.2f m", distance_traveled_);

                    feedback->current_error_x = error;

                    double linear_speed = Kp_linear * error;
                    double max_speed = 0.12;
                    // linear_speed = std::clamp(linear_speed, -max_speed, max_speed);

                    if(linear_speed < -max_speed){
                        linear_speed = -max_speed;
                    }
        
                    if(linear_speed > max_speed){
                        linear_speed = max_speed;
                    }
                    // RCLCPP_INFO(this->get_logger(), "Velocidade linear : %.2f m/s", linear_speed);
                    twist_msg.linear.x = linear_speed;
                    cmd_vel_publisher_->publish(twist_msg);

                    if (std::abs(error) < linear_tolerance)  // Tolerância de 0.03 m
                    {
                        twist_msg.linear.x = 0.0;
                        cmd_vel_publisher_->publish(twist_msg);
                        start_movement_ = false;
                        move_forward_ = false;
                        finish_action = true;
                        RCLCPP_INFO(this->get_logger(), "Reto finish %d", finish_action);
                        result->success = true;
                        goal_handle->succeed(result);
                        return;
                    }
                }
            }
            if(start_movement_ || move_forward_){
                feedback->current_error_x = error;
                feedback->current_error_yaw = error_angle;

                goal_handle->publish_feedback(feedback);
            }
        }
    }

    // Callback of detect dock service
    void dock_detection_callback(rclcpp::Client<drn_perception_msgs::srv::DocksDetected>::SharedFuture future)
    {
        if (future.get()->dock_detected)
        {
            if (dock_index >= 0 && dock_index < static_cast<int>(future.get()->docks.size()))
            {
                auto dock = future.get()->docks[dock_index];
                tf2::Quaternion q(
                    dock.pose_stamped.pose.orientation.x,
                    dock.pose_stamped.pose.orientation.y,
                    dock.pose_stamped.pose.orientation.z,
                    dock.pose_stamped.pose.orientation.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                RCLCPP_INFO(this->get_logger(), "Dock detected at position: (%.2f, %.2f, %.2f), Yaw: %.2f rad (%.2f deg), frame %s",
                            dock.pose_stamped.pose.position.x, dock.pose_stamped.pose.position.y, dock.pose_stamped.pose.position.z, yaw, yaw * 180.0 / M_PI, dock.pose_stamped.header.frame_id.c_str());
                
                auto adjusted_pose = compute_point2(dock.pose_stamped);
                
                tf2::Quaternion q_adj;
                tf2::fromMsg(adjusted_pose.pose.orientation, q_adj);
                double roll_adj, pitch_adj, yaw_adj;
                tf2::Matrix3x3(q_adj).getRPY(roll_adj, pitch_adj, yaw_adj);
                
                RCLCPP_INFO(this->get_logger(), "Adjusted pose: (%.2f, %.2f, %.2f), Yaw: %.2f rad (%.2f deg) - Frame: %s",
                            adjusted_pose.pose.position.x, adjusted_pose.pose.position.y, adjusted_pose.pose.position.z,
                            yaw_adj, yaw_adj * 180.0 / M_PI, adjusted_pose.header.frame_id.c_str());
                
                navigate_to_dock(adjusted_pose);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Índice de dock inválido: %d", dock_index);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Nenhum dock detectado.");
            not_dock = true;
        }
    }

    // Adjust pose in relation base_link with front_lidar
    geometry_msgs::msg::PoseStamped compute_point2(const geometry_msgs::msg::PoseStamped &point1)
    {
        geometry_msgs::msg::TransformStamped transform_bf;
       
        try
        {
            if (tf_buffer_1->canTransform("base_link", "front_lidar", tf2::TimePointZero))
            {
                transform_bf = tf_buffer_1->lookupTransform("base_link", "front_lidar", tf2::TimePointZero);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "A transformação entre base_link e front_lidar não está disponível.");
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Falha ao obter transformação: %s", ex.what());
        }

        double dx = transform_bf.transform.translation.x+ distance_seg;
        double dy = transform_bf.transform.translation.y;

        tf2::Quaternion q(point1.pose.orientation.x, 
                  point1.pose.orientation.y, 
                  point1.pose.orientation.z, 
                  point1.pose.orientation.w);
        
        tf2::Matrix3x3 rot_matrix(q);
        // RCLCPP_INFO(rclcpp::get_logger("debug"), "Rotação: \n%f %f %f\n%f %f %f\n%f %f %f",
        //     rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2],
        //     rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2],
        //     rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2]);

        double rotated_dx = rot_matrix[0][0] * dx + rot_matrix[0][1] * dy;
        double rotated_dy = rot_matrix[1][0] * dx + rot_matrix[1][1] * dy;

        // RCLCPP_INFO(rclcpp::get_logger("debug"), "dx: %f, dy: %f", dx, dy);
        // RCLCPP_INFO(rclcpp::get_logger("debug"), "rotated_dx: %f, rotated_dy: %f", rotated_dx, rotated_dy);

        geometry_msgs::msg::PoseStamped adjusted_pose = point1;
        adjusted_pose.pose.position.x -= rotated_dx;
        adjusted_pose.pose.position.y -= rotated_dy;
        
        return adjusted_pose;  
    }
    
    //Init variables navigate to dock
    void navigate_to_dock(const geometry_msgs::msg::PoseStamped &goal_pose)
    {
        RCLCPP_INFO(this->get_logger(), "Adjust moviment, goo ...");
        start_position_ = last_position_;
        start_movement_ = true;
        first_time = true;

        double dgoal_x;
        double dgoal_y;

        if (direction == "front")
        {
            if(goal_pose.pose.position.x < 0){
                dgoal_x = - goal_pose.pose.position.x;
                dgoal_y = - goal_pose.pose.position.y;
                target_angle = std::atan2(dgoal_y, dgoal_x);
                RCLCPP_INFO(this->get_logger(), "Entrei FRONT 1Q Target angle : %.2f rad", target_angle);
                distance_to_target = - std::sqrt(dgoal_x * dgoal_x + dgoal_y * dgoal_y);
                RCLCPP_INFO(this->get_logger(), "Entrei FRONT 1Q Distance target : %.2f m", distance_to_target);
            }else{
                dgoal_x = goal_pose.pose.position.x;
                dgoal_y = goal_pose.pose.position.y;
                target_angle = std::atan2(dgoal_y, dgoal_x);
                RCLCPP_INFO(this->get_logger(), "Entrei FRONT 2Q Target angle : %.2f rad", target_angle);
                distance_to_target = std::sqrt(dgoal_x * dgoal_x + dgoal_y * dgoal_y);
                RCLCPP_INFO(this->get_logger(), "Entrei FRONT 2Q Distance target : %.2f m", distance_to_target);
            }
        }else{
            if (direction == "rear")
            {
                if(goal_pose.pose.position.x < 0){
                    dgoal_x = - goal_pose.pose.position.x;
                    dgoal_y = - goal_pose.pose.position.y;
                    target_angle = std::atan2(dgoal_y, dgoal_x);
                    RCLCPP_INFO(this->get_logger(), "Entrei REAR 2Q Target angle : %.2f rad", target_angle);
                    distance_to_target = - std::sqrt(dgoal_x * dgoal_x + dgoal_y * dgoal_y);
                    RCLCPP_INFO(this->get_logger(), "Entrei REAR 2Q Distance target : %.2f m", distance_to_target);
                }else{
                    dgoal_x = goal_pose.pose.position.x;
                    dgoal_y = goal_pose.pose.position.y;
                    target_angle = std::atan2(dgoal_y, dgoal_x);
                    RCLCPP_INFO(this->get_logger(), "Entrei REAR 1Q Target angle : %.2f rad", target_angle);
                    distance_to_target = std::sqrt(dgoal_x * dgoal_x + dgoal_y * dgoal_y);
                    RCLCPP_INFO(this->get_logger(), "Entrei REAR 1Q Distance target : %.2f m", distance_to_target);
                }    
            }
        }
        
        // RCLCPP_INFO(this->get_logger(), "Target angle : %.2f rad", target_angle);
        // RCLCPP_INFO(this->get_logger(), "Distance target : %.2f m", distance_to_target);
        Kp_linear = 1.0; 
        Kp_angular = 1.0;
    }
    
    //Callback odom with navigate to dock
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_position_.pose = msg->pose.pose;

        if(first_time){
            start_position_ = last_position_;
            first_time = false;
        }        
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init the docking action.");
  auto node = std::make_shared<DockNavigationServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  RCLCPP_INFO(node->get_logger(), "Keyboard interrupt, shutting down.\n");
  rclcpp::shutdown();
  return 0;
}
