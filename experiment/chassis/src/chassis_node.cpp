//
// Created by yenkn on 2020/9/27.
//
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sandbox_msgs/ChassisCommand.h>
#include <hal/tiny_serial.h>
#include <TinyProtocol.h>
#include <thread>
#include <functional>

ros::Publisher *velocity_publisher_;

#define MSG_TYPE_TWIST 0xe1
#define MSG_TYPE_PID 0xe2
#define MSG_TYPE_VELOCITY 0xe3

struct TwistMessage {
  int16_t velocity;
  int16_t angle;
};

struct PIDMessage {
  float kp;
  float ki;
};

struct VelocityMessage {
  int16_t velocity;
};

typedef unsigned char byte;

void handle_message(byte type, byte *data, int len) {
  if(type == MSG_TYPE_VELOCITY) {
    if(len != sizeof(VelocityMessage)) return;
    VelocityMessage *vel = (VelocityMessage *)data;

    std_msgs::Float64 msg;
    msg.data = vel->velocity / 100.0;
    velocity_publisher_->publish(msg);
  }
}

int16_t generate_message(byte target, byte type, const byte *data, int16_t len, byte *out) {
  int16_t pos = 0, dpos = 0;

  out[pos++] = target;
  out[pos++] = type;

  while(dpos < len) {
    out[pos++] = data[dpos++];
  }

  return pos;
}

tiny_serial_handle_t port_handle;
Tiny::ProtoLight proto;

void cmd_vel_cb(const sandbox_msgs::ChassisCommandConstPtr &msg) {
  // 夹值与推演器(map_param/multi_vehicle_config)一致:转向 = max_steer_angle 0.50rad = 28.65°;
  // 油门 = max_speed 0.26m/s ×100 = 26。否则最紧弯/最高速跟不上规划轨迹。
  double steering = std::max(std::min(-msg->steering / M_PI * 180, 28.65), -28.65);
  double throttle = std::max(std::min(msg->throttle * 100, 26.0), -26.0);
  TwistMessage twist = {
      int16_t (throttle),
      int16_t (steering),
  };

  byte buf[100] = { 0 };
  int16_t size = generate_message((unsigned char)msg->target, MSG_TYPE_TWIST, (byte *)&twist, sizeof(TwistMessage), buf);
  proto.write(reinterpret_cast<char *>(buf), size);
}

int serial_send_fd(void *p, const void *buf, int len) {
  return tiny_serial_send(port_handle, buf, len);
}

int serial_receive_fd(void *p, void *buf, int len) {
  return tiny_serial_read(port_handle, buf, len);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "chassis_node");
  ros::NodeHandle nh;

  // port, baudrate, timeout in milliseconds
  port_handle = tiny_serial_open("/dev/ttyUSB0", 57600);
  if (port_handle == TINY_SERIAL_INVALID)
  {
    std::cerr <<  "Error opening serial port" << std::endl;
    return 1;
  }
  proto.enableCheckSum();
  proto.begin(serial_send_fd, serial_receive_fd);

  bool is_terminate = false;
  std::thread rx_thread([&is_terminate](Tiny::ProtoLight &proto) {
    Tiny::Packet<12> packet;
    while (!is_terminate) {
      int len = proto.read(packet);
      if (len > 1) {
        unsigned char *data = (unsigned char *)packet.data();
        handle_message(data[0], data + 1, len - 1);
      }
    }
  }, std::ref(proto));

  ros::Subscriber cmd_subscriber = nh.subscribe("/chassis", 10, cmd_vel_cb);
  ros::Publisher vel_publisher = nh.advertise<std_msgs::Float64>("/velocity", 1, false);
  velocity_publisher_ = &vel_publisher;

  ros::spin();
  is_terminate = true;
  rx_thread.join();
  proto.end();

  tiny_serial_close(port_handle);
  return 0;
}