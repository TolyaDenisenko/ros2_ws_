// Нода связи с ESP32 (протокол v2, include/esp32_bridge/protocol.hpp):
//   ESP32 → /scan (sensor_msgs/LaserScan), /odom + TF odom→base_link
//   /cmd_vel (geometry_msgs/Twist) → ESP32 (скорости колёс, мм/с)
//   сервис /reset_odom (std_srvs/Trigger) — обнулить одометрию на ESP32
//
// Транспорт (параметр transport):
//   udp    — Wi-Fi, ESP32 по адресу host:port. ESP отвечает тому, кто шлёт
//            ей пакеты, поэтому нода с самого старта пингует её 2 раза в секунду.
//   serial — USB, устройство device (по умолчанию /dev/ttyACM0), пакеты в COBS.
//
// Метки времени: ESP ставит в каждый пакет своё время, нода переводит его
// в своё по синхронизации часов (PING/PONG). Пока синхронизации нет —
// берётся время прихода пакета.
//
// Потоки: ROS-колбэки — в главном потоке, весь обмен с ESP32 — в потоке
// io_context. Сокет/порт трогаются только из него; команды туда — через asio::post.

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "esp32_bridge/protocol.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace asio = boost::asio;
using udp = asio::ip::udp;
using namespace std::chrono_literals;

namespace esp32_bridge
{

// =============================================================
//    Транспорт: принимает «сырые» пакеты протокола, отправляет их
// =============================================================
class Transport
{
public:
  using OnPacket = std::function<void (const uint8_t *, std::size_t)>;
  virtual ~Transport() = default;
  virtual void start(OnPacket on_packet) = 0;               // в потоке io_context
  virtual void send(const std::vector<uint8_t> & pkt) = 0;  // в потоке io_context
};

// UDP по Wi-Fi: одна датаграмма = один пакет
class UdpTransport : public Transport
{
public:
  UdpTransport(asio::io_context & io, const std::string & host, uint16_t port, rclcpp::Logger log)
  : socket_(io, udp::endpoint(udp::v4(), 0)), log_(log)
  {
    udp::resolver resolver(io);
    peer_ = *resolver.resolve(udp::v4(), host, std::to_string(port)).begin();
  }

  void start(OnPacket on_packet) override
  {
    on_packet_ = std::move(on_packet);
    receive();
  }

  void send(const std::vector<uint8_t> & pkt) override
  {
    boost::system::error_code ec;
    socket_.send_to(asio::buffer(pkt), peer_, 0, ec);   // UDP не блокирует надолго
    if (ec) {
      RCLCPP_WARN_THROTTLE(log_, clock_, 2000, "UDP send: %s", ec.message().c_str());
    }
  }

private:
  void receive()
  {
    socket_.async_receive_from(asio::buffer(rx_), sender_,
      [this](boost::system::error_code ec, std::size_t n) {
        // Принимаем только от ESP32 (адрес из параметра host)
        if (!ec && sender_.address() == peer_.address()) {
          on_packet_(rx_.data(), n);
        }
        receive();
      });
  }

  udp::socket socket_;
  udp::endpoint peer_, sender_;
  std::array<uint8_t, 2048> rx_{};
  OnPacket on_packet_;
  rclcpp::Logger log_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

// USB (виртуальный COM-порт): поток байт, пакеты в COBS с разделителем 0x00.
// Если устройство пропало (кабель выдернули, ESP перезагрузилась) —
// раз в секунду пытается открыть его снова.
class SerialTransport : public Transport
{
public:
  SerialTransport(asio::io_context & io, std::string device, unsigned baud, rclcpp::Logger log)
  : port_(io), timer_(io), device_(std::move(device)), baud_(baud), log_(log) {}

  void start(OnPacket on_packet) override
  {
    on_packet_ = std::move(on_packet);
    open();
  }

  void send(const std::vector<uint8_t> & pkt) override
  {
    if (!port_.is_open()) {
      return;
    }
    auto frame = cobs_encode(pkt);
    frame.push_back(0);
    boost::system::error_code ec;
    asio::write(port_, asio::buffer(frame), ec);
    if (ec) {
      fail("write", ec);
    }
  }

private:
  void open()
  {
    boost::system::error_code ec;
    port_.open(device_, ec);
    if (!ec) {
      // 8N1 без управления потоком (для встроенного USB ESP32-S3 скорость не важна)
      using sp = asio::serial_port_base;
      port_.set_option(sp::baud_rate(baud_), ec);
      port_.set_option(sp::character_size(8), ec);
      port_.set_option(sp::parity(sp::parity::none), ec);
      port_.set_option(sp::stop_bits(sp::stop_bits::one), ec);
      port_.set_option(sp::flow_control(sp::flow_control::none), ec);
    }
    if (ec) {
      RCLCPP_WARN_THROTTLE(log_, clock_, 5000, "Не открыть %s: %s", device_.c_str(),
        ec.message().c_str());
      return retry();
    }
    RCLCPP_INFO(log_, "Открыт %s", device_.c_str());
    stream_ = CobsStream{};
    read();
  }

  void read()
  {
    port_.async_read_some(asio::buffer(rx_), [this](boost::system::error_code ec, std::size_t n) {
        if (ec) {
          return fail("read", ec);
        }
        for (const auto & pkt : stream_.feed(rx_.data(), n)) {
          on_packet_(pkt.data(), pkt.size());
        }
        read();
      });
  }

  void fail(const char * what, boost::system::error_code ec)
  {
    if (!port_.is_open()) {
      return;   // уже обрабатываем потерю порта
    }
    RCLCPP_WARN(log_, "%s %s: %s — переподключение", device_.c_str(), what, ec.message().c_str());
    boost::system::error_code ignore;
    port_.close(ignore);
    retry();
  }

  void retry()
  {
    timer_.expires_after(1s);
    timer_.async_wait([this](boost::system::error_code) {open();});
  }

  asio::serial_port port_;
  asio::steady_timer timer_;
  std::string device_;
  unsigned baud_;
  std::array<uint8_t, 4096> rx_{};
  CobsStream stream_;
  OnPacket on_packet_;
  rclcpp::Logger log_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

// =============================================================
//                          Нода
// =============================================================
class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : Node("esp_bridge")
  {
    // ── связь ───────────────────────────────────────────────────
    const auto transport = declare_parameter("transport", "udp");     // udp | serial
    const auto host = declare_parameter("host", "192.168.0.105");     // udp: адрес ESP32
    const auto port = declare_parameter("port", 8888);                // udp: порт ESP32
    const auto device = declare_parameter("device", "/dev/ttyACM0");  // serial: устройство
    const auto baud = declare_parameter("baud", 921600);              // serial: скорость

    // ── кадры и лидар ────────────────────────────────────────────
    frame_id_ = declare_parameter("frame_id", "laser");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    range_min_ = declare_parameter("range_min", 0.05);
    range_max_ = declare_parameter("range_max", 16.0);
    scan_opt_.yaw_deg = declare_parameter("lidar_yaw_deg", 180.0);   // поворот облака, °
    scan_opt_.mirror = declare_parameter("lidar_mirror", true);      // зеркально
    scan_opt_.bins = static_cast<std::size_t>(declare_parameter("scan_bins", 360));
    const auto xyz = declare_parameter("lidar_xyz", std::vector<double>{0.10, 0.0, 0.17});
    const auto rpy = declare_parameter("lidar_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});

    // ── робот и управление ───────────────────────────────────────
    wheel_base_ = declare_parameter("wheel_base", 0.54);   // колея, м = WHEEL_BASE в прошивке
    // Последняя команда повторяется 20 раз в секунду (UDP может терять пакеты),
    // но не дольше cmd_timeout после последнего /cmd_vel — потом робот встанет.
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);

    // Нулевой map→odom — только для RViz без SLAM. Со SLAM не включать:
    // он перебивает поправку SLAM, и карта перестаёт замыкаться.
    const bool publish_map_odom = declare_parameter("publish_map_odom_tf", false);

    // ── ROS-интерфейсы ───────────────────────────────────────────
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_cmd_vel(*msg);});
    reset_srv_ = create_service<std_srvs::srv::Trigger>("/reset_odom",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res) {
          post_send(type::kResetOdom, {});
          res->success = true;
          res->message = "RESET_ODOM отправлен";
        });
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publish_static_tf(xyz, rpy, publish_map_odom);

    // ── транспорт ────────────────────────────────────────────────
    if (transport == "udp") {
      link_ = std::make_unique<UdpTransport>(io_, host, static_cast<uint16_t>(port), get_logger());
      RCLCPP_INFO(get_logger(), "Связь с ESP32: UDP %s:%ld", host.c_str(), port);
    } else if (transport == "serial") {
      link_ = std::make_unique<SerialTransport>(io_, device, static_cast<unsigned>(baud),
          get_logger());
      RCLCPP_INFO(get_logger(), "Связь с ESP32: USB %s", device.c_str());
    } else {
      throw std::invalid_argument("transport должен быть udp или serial, а не " + transport);
    }

    asio::post(io_, [this] {
        link_->start([this](const uint8_t * d, std::size_t n) {on_packet(d, n);});
      });
    work_ = std::make_unique<asio::executor_work_guard<asio::io_context::executor_type>>(
      io_.get_executor());
    net_thread_ = std::thread([this] {io_.run();});

    // PING: синхронизация часов и «я здесь» для ESP (UDP)
    ping_timer_ = create_wall_timer(500ms, [this] {post_send(type::kPing, {});});
    // Повтор последней команды
    cmd_timer_ = create_wall_timer(50ms, [this] {resend_cmd();});
    // Статистика связи
    stats_timer_ = create_wall_timer(10s, [this] {log_stats();});
  }

  ~BridgeNode() override
  {
    work_.reset();
    io_.stop();
    if (net_thread_.joinable()) {
      net_thread_.join();
    }
  }

private:
  // Время ПК в наносекундах — те же часы, что у ROS-сообщений (system time)
  int64_t pc_now_ns() {return now().nanoseconds();}

  // Отправить пакет из любого потока: собирается и уходит в потоке io_context.
  // stamp у пакетов ПК — время ПК; для PING ESP вернёт его в PONG.
  void post_send(uint8_t t, std::vector<uint8_t> payload)
  {
    const uint64_t stamp = static_cast<uint64_t>(pc_now_ns());
    asio::post(io_, [this, t, stamp, payload = std::move(payload)] {
        link_->send(build_packet(t, tx_seq_++, stamp, payload.data(), payload.size()));
      });
  }

  // =============================================================
  //              входящие пакеты (поток io_context)
  // =============================================================
  void on_packet(const uint8_t * data, std::size_t len)
  {
    const auto pkt = parse_packet(data, len);
    if (!pkt) {
      ++crc_errors_;
      return;
    }
    {
      std::lock_guard<std::mutex> lk(stats_mutex_);
      loss_.add(pkt->seq);
    }
    switch (pkt->type) {
      case type::kOdom: publish_odom(*pkt); break;
      case type::kScan: on_scan_chunk(*pkt); break;
      case type::kPong: on_pong(*pkt); break;
      case type::kLog:
        RCLCPP_INFO(get_logger(), "[esp32] %s",
          std::string(pkt->payload.begin(), pkt->payload.end()).c_str());
        break;
      default: break;   // неизвестные типы — для совместимости с будущими прошивками
    }
  }

  void on_pong(const Packet & pkt)
  {
    if (pkt.payload.size() != 8) {
      return;
    }
    int64_t sent = 0;
    for (int i = 7; i >= 0; --i) {
      sent = (sent << 8) | pkt.payload[i];
    }
    std::lock_guard<std::mutex> lk(sync_mutex_);
    sync_.add(sent, pc_now_ns(), pkt.stamp);
  }

  // Время ESP (мкс) → время ROS. Без синхронизации — время прихода.
  rclcpp::Time esp_to_ros(uint64_t esp_us)
  {
    std::lock_guard<std::mutex> lk(sync_mutex_);
    if (const auto ns = sync_.to_pc_ns(esp_us)) {
      return rclcpp::Time(*ns, RCL_ROS_TIME);
    }
    return now();
  }

  void on_scan_chunk(const Packet & pkt)
  {
    const auto chunk = parse_scan_chunk(pkt.payload);
    if (!chunk) {
      return;
    }
    const auto rev = assembler_.add(*chunk, pkt.stamp);
    if (!rev) {
      return;
    }
    if (!rev->complete) {
      ++incomplete_scans_;
    }
    const Scan s = bin_scan(rev->frames, scan_opt_);

    sensor_msgs::msg::LaserScan msg;
    // Метка — середина оборота: точки сняты за всё время между первым и последним куском
    msg.header.stamp = esp_to_ros((rev->first_stamp_us + rev->last_stamp_us) / 2);
    msg.header.frame_id = frame_id_;
    msg.angle_min = s.angle_min;
    msg.angle_max = s.angle_max;
    msg.angle_increment = s.angle_increment;
    msg.scan_time = static_cast<float>((rev->last_stamp_us - rev->first_stamp_us) * 1e-6);
    msg.range_min = static_cast<float>(range_min_);
    msg.range_max = static_cast<float>(range_max_);
    msg.ranges = s.ranges;
    msg.intensities = s.intensities;
    scan_pub_->publish(msg);
  }

  void publish_odom(const Packet & pkt)
  {
    const auto o = parse_odom(pkt.payload);
    if (!o) {
      return;
    }
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, o->th);

    nav_msgs::msg::Odometry od;
    od.header.stamp = esp_to_ros(pkt.stamp);
    od.header.frame_id = odom_frame_;
    od.child_frame_id = base_frame_;
    od.pose.pose.position.x = o->x;
    od.pose.pose.position.y = o->y;
    od.pose.pose.orientation.x = q.x();
    od.pose.pose.orientation.y = q.y();
    od.pose.pose.orientation.z = q.z();
    od.pose.pose.orientation.w = q.w();
    od.twist.twist.linear.x = (o->speed_left_mm + o->speed_right_mm) / 2000.0;
    od.twist.twist.angular.z = (o->speed_right_mm - o->speed_left_mm) / (wheel_base_ * 1000.0);
    odom_pub_->publish(od);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = od.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = o->x;
    tf.transform.translation.y = o->y;
    tf.transform.rotation = od.pose.pose.orientation;
    tf_br_->sendTransform(tf);
  }

  // =============================================================
  //                 команды (главный поток ROS)
  // =============================================================
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg)
  {
    last_cmd_ = wheel_speeds_mm(msg.linear.x, msg.angular.z, wheel_base_);
    last_cmd_time_ = now();
    post_send(type::kCmd, encode_command(last_cmd_[0], last_cmd_[1]));
  }

  void resend_cmd()
  {
    if (last_cmd_time_.nanoseconds() == 0 ||
      (now() - last_cmd_time_).seconds() > cmd_timeout_)
    {
      return;   // команд давно нет — ESP сама остановит колёса по своему таймауту
    }
    post_send(type::kCmd, encode_command(last_cmd_[0], last_cmd_[1]));
  }

  void log_stats()
  {
    uint64_t rx, lost;
    {
      std::lock_guard<std::mutex> lk(stats_mutex_);
      rx = loss_.received();
      lost = loss_.lost();
    }
    std::optional<int64_t> rtt;
    {
      std::lock_guard<std::mutex> lk(sync_mutex_);
      rtt = sync_.best_rtt_ns();
    }
    if (rx == 0) {
      RCLCPP_WARN(get_logger(), "От ESP32 ничего не пришло");
      return;
    }
    RCLCPP_INFO(get_logger(),
      "Связь: принято %lu, потеряно %lu (%.1f%%), ошибок CRC %lu, неполных оборотов %lu, RTT %.1f мс",
      rx, lost, 100.0 * lost / (rx + lost), crc_errors_.load(), incomplete_scans_.load(),
      rtt ? *rtt / 1e6 : -1.0);
  }

  void publish_static_tf(
    const std::vector<double> & xyz, const std::vector<double> & rpy_deg, bool map_odom)
  {
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    geometry_msgs::msg::TransformStamped laser;
    laser.header.stamp = now();
    laser.header.frame_id = base_frame_;
    laser.child_frame_id = frame_id_;
    laser.transform.translation.x = xyz.at(0);
    laser.transform.translation.y = xyz.at(1);
    laser.transform.translation.z = xyz.at(2);
    tf2::Quaternion q;
    q.setRPY(rpy_deg.at(0) * M_PI / 180.0, rpy_deg.at(1) * M_PI / 180.0,
      rpy_deg.at(2) * M_PI / 180.0);
    laser.transform.rotation.x = q.x();
    laser.transform.rotation.y = q.y();
    laser.transform.rotation.z = q.z();
    laser.transform.rotation.w = q.w();
    tfs.push_back(laser);

    if (map_odom) {
      geometry_msgs::msg::TransformStamped mo;
      mo.header.stamp = laser.header.stamp;
      mo.header.frame_id = "map";
      mo.child_frame_id = odom_frame_;
      mo.transform.rotation.w = 1.0;
      tfs.push_back(mo);
      RCLCPP_WARN(get_logger(), "Публикуется нулевой TF map→odom — не запускайте вместе с SLAM");
    }
    static_br_->sendTransform(tfs);
  }

  // параметры
  std::string frame_id_, odom_frame_, base_frame_;
  double range_min_, range_max_, wheel_base_, cmd_timeout_;
  ScanOptions scan_opt_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr ping_timer_, cmd_timer_, stats_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;

  // команды (главный поток)
  std::array<int16_t, 2> last_cmd_{};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  // связь (поток io_context, кроме полей под мьютексами и atomic)
  asio::io_context io_;
  std::unique_ptr<asio::executor_work_guard<asio::io_context::executor_type>> work_;
  std::unique_ptr<Transport> link_;
  std::thread net_thread_;
  uint16_t tx_seq_ = 0;
  ScanAssembler assembler_;
  std::mutex sync_mutex_, stats_mutex_;
  TimeSync sync_;
  LossCounter loss_;
  std::atomic<uint64_t> crc_errors_{0}, incomplete_scans_{0};
};

}  // namespace esp32_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<esp32_bridge::BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
