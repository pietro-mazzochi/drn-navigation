#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <utility>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/execution_timer.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/robot_utils.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <drn_navigation_msgs/srv/get_costs.hpp>

constexpr unsigned char LETHAL_OBSTACLE = 254;
constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
constexpr unsigned char NO_INFORMATION = 255;
constexpr unsigned char MAX_NON_OBSTACLE = 252;
constexpr unsigned char FREE_SPACE = 0;

using namespace std::placeholders;
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::Costmap2DROS;

class GetCostService : public rclcpp::Node
{
public:
  GetCostService()
  : Node("get_cost_service")
  { 
    RCLCPP_INFO(this->get_logger(), "Inicializando GetCostService...");
    
    //Subcripção ao topico global_costmap 
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap",
      10,
      std::bind(&GetCostService::costmapCallback, this, std::placeholders::_1));
    
    footprint_subscription_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/global_costmap/footprint",
      10,
      std::bind(&GetCostService::footprintCallback, this, std::placeholders::_1));

    // Iniciar o servidor de serviço
    service_ = this->create_service<drn_navigation_msgs::srv::GetCosts>(
      "/global_costmap/get_cost_global_costmap",
      std::bind(&GetCostService::getCostsCallback, this, _1, _2, _3));

    RCLCPP_INFO(this->get_logger(), "Serviço '/global_costmap/get_cost_global_costmap' iniciado.");
  }

private:
  std::shared_ptr<Costmap2DROS> costmap_ros_;
  rclcpp::Service<drn_navigation_msgs::srv::GetCosts>::SharedPtr service_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_subscription_;
  nav2_costmap_2d::Costmap2D costmap;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::PolygonStamped current_footprint_;
  std::mutex footprint_mutex_;

  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  current_footprint_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Footprint atualizado com %lu pontos.", current_footprint_.polygon.points.size());
  }
  
  bool transformPoseToGlobalFrame(
    const geometry_msgs::msg::PoseStamped &input_pose,
    geometry_msgs::msg::PoseStamped &transformed_pose,
    const std::string &global_frame,
    const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
    const rclcpp::Duration transform_tolerance)
  {
    if (input_pose.header.frame_id == global_frame) {
      transformed_pose = input_pose;
      return true;
    }

    try {
      transformed_pose = tf_buffer->transform(
        input_pose, global_frame, tf2::durationFromSec(transform_tolerance.seconds()));
      return true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Falha ao transformar pose: %s", ex.what());
      return false;
    }
  }
  
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    costmap.resizeMap(msg->info.width, msg->info.height, msg->info.resolution,
                      msg->info.origin.position.x, msg->info.origin.position.y);
    
    // Transferir os dados do OccupancyGrid para o Costmap2D
    unsigned char *costmap_data = costmap.getCharMap();
    for (size_t i = 0; i < msg->data.size(); i++) {
      costmap_data[i] = static_cast<unsigned char>(msg->data[i]);
    }
    // RCLCPP_INFO(this->get_logger(), "Costmap carregado com %d x %d células.", costmap.getSizeInCellsX(), costmap.getSizeInCellsY());
  }

  void getCostsCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<drn_navigation_msgs::srv::GetCosts::Request> request,
    const std::shared_ptr<drn_navigation_msgs::srv::GetCosts::Response> response)
  {
    unsigned int mx, my;
  
    // RCLCPP_INFO(this->get_logger(), "Processando as poses recebidas...");

    for (const auto & pose : request->poses) {
      // RCLCPP_INFO(this->get_logger(), "Processando pose: (%f, %f)", pose.pose.position.x, pose.pose.position.y);

      geometry_msgs::msg::PoseStamped pose_transformed;
      //costmap.transformPoseToGlobalFrame(pose, pose_transformed);
      if (!transformPoseToGlobalFrame(
        pose, pose_transformed,"map", tf_buffer_, rclcpp::Duration::from_seconds(0.1))) {
        RCLCPP_ERROR(this->get_logger(), "Falha ao transformar a pose. Continuando para a próxima pose...");
        response->costs.push_back(NO_INFORMATION);
        continue;
      }
     
      bool in_bounds = costmap.worldToMap(
        pose_transformed.pose.position.x,
        pose_transformed.pose.position.y, mx, my);

      if (!in_bounds) {
        RCLCPP_WARN(this->get_logger(), "Pose fora dos limites do mapa, retornando 'NO_INFORMATION'.");
        response->costs.push_back(NO_INFORMATION);
        continue;
      }

      double yaw = tf2::getYaw(pose_transformed.pose.orientation);
      // RCLCPP_INFO(this->get_logger(), "Pose transformada para o frame global: (%f, %f), yaw: %f", 
      //             pose_transformed.pose.position.x, pose_transformed.pose.position.y, yaw);

      if (request->use_footprint) {
        // RCLCPP_INFO(this->get_logger(), "Calculando custo com footprint...");
        
        std::vector<geometry_msgs::msg::Point> footprint;
        std::lock_guard<std::mutex> lock(footprint_mutex_);
        for (const auto &point : current_footprint_.polygon.points) {
          geometry_msgs::msg::Point pt;
          pt.x = point.x;
          pt.y = point.y;
          pt.z = point.z;
          footprint.push_back(pt);
        }
      
        nav2_costmap_2d::FootprintCollisionChecker<Costmap2D *> collision_checker(&costmap);

        float cost = collision_checker.footprintCostAtPose(
        pose_transformed.pose.position.x,
        pose_transformed.pose.position.y, yaw, footprint);

        // RCLCPP_INFO(this->get_logger(), "Custo calculado: %f", cost);
        response->costs.push_back(cost);
      } else {
        float cost = static_cast<float>(costmap.getCost(mx, my));
        // RCLCPP_INFO(this->get_logger(), "Custo calculado s/footprint: %f", cost);
        response->costs.push_back(cost);
      }
    }

    // RCLCPP_INFO(this->get_logger(), "Resposta com custos gerada com sucesso.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inicializando o nó GetCostService.");
  auto node = std::make_shared<GetCostService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
