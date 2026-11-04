// /scan (LaserScan) → /scan_cloud (PointCloud2) для octomap_server.
// Облако остаётся в кадре лидара: octomap_server сам переводит его в кадр
// карты через TF и берёт позицию лидара как начало лучей.
//
// Параметры:
//   scan_topic  (по умолчанию scan)
//   cloud_topic (по умолчанию scan_cloud)

#include <memory>

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class ScanToCloud : public rclcpp::Node
{
public:
  ScanToCloud()
  : Node("scan_to_cloud")
  {
    const auto scan_topic = declare_parameter("scan_topic", "scan");
    const auto cloud_topic = declare_parameter("cloud_topic", "scan_cloud");

    // Публикуем с reliable (QoS по умолчанию): совместимо и с reliable-,
    // и с best-effort-подписчиками.
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, 5);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        sensor_msgs::msg::PointCloud2 cloud;
        // Лучи без возврата (inf/NaN) projectLaser отбрасывает сам
        projector_.projectLaser(*scan, cloud);
        pub_->publish(cloud);
      });
  }

private:
  laser_geometry::LaserProjection projector_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToCloud>());
  rclcpp::shutdown();
  return 0;
}
