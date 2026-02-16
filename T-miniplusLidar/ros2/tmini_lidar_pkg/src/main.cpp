#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class TMiniLidarNode : public rclcpp::Node {
public:
    TMiniLidarNode() : Node("tmini_lidar_node"), step(0), si_index(0) {
        // Publisher 설정
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        
        // 시리얼 포트 설정
        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "포트 /dev/ttyUSB0를 열 수 없습니다!");
            return;
        }

        struct termios tty;
        tcgetattr(fd, &tty);
        cfsetospeed(&tty, B230400); cfsetispeed(&tty, B230400);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
        tty.c_lflag = 0; tty.c_oflag = 0; tty.c_iflag = 0;
        tcsetattr(fd, TCSANOW, &tty);

        // 리다 시작 명령 송신
        unsigned char start_cmd[] = {0xA5, 0x60};
        write(fd, start_cmd, 2);

        // 360도 배열 초기화
        ranges.assign(360, std::numeric_limits<float>::infinity());
        intensities.assign(360, 0.0f);
        
        timer_ = this->create_wall_timer(10ms, std::bind(&TMiniLidarNode::process_serial, this));
    }

private:
    void process_serial() {
        uint8_t rxtemp;
        while (read(fd, &rxtemp, 1) > 0) {
            switch (step) {
                case 0: if (rxtemp == 0xAA) { g_recvbuf[0] = rxtemp; step = 1; } break;
                case 1: if (rxtemp == 0x55) { g_recvbuf[1] = rxtemp; step = 2; } else step = 0; break;
                case 2: g_recvbuf[2] = rxtemp; step = 3; break;
                case 3: g_recvbuf[3] = rxtemp; si_len = rxtemp * 3; step = 4; break;
                case 4: case 5: case 6: case 7: case 8: case 9: g_recvbuf[step++] = rxtemp; break;
                case 10:
                    g_recvbuf[10 + si_index++] = rxtemp;
                    if (si_index >= si_len) {
                        update_and_publish(); 
                        step = 0; si_index = 0;
                    }
                    break;
            }
        }
    }

    void update_and_publish() {
        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.stamp = this->now();
        msg.header.frame_id = "laser_frame";
        msg.angle_min = 0.0;
        msg.angle_max = 2.0 * M_PI;
        msg.angle_increment = (2.0 * M_PI) / 360.0;
        msg.range_min = 0.1;
        msg.range_max = 12.0;

        // 메시지 배열 크기 정합성 확보
        msg.ranges.assign(360, std::numeric_limits<float>::infinity());
        msg.intensities.assign(360, 0.0f);

        uint16_t FSA = g_recvbuf[5] << 8 | g_recvbuf[4];
        uint16_t LSA = g_recvbuf[7] << 8 | g_recvbuf[6];
        double s_ang = (double)((FSA >> 1) / 64.0);
        double e_ang = (double)((LSA >> 1) / 64.0);
        uint8_t lsn = g_recvbuf[3];

        // 각도 차이 및 증분 계산 (SDK 정밀도 반영)
        double diff = e_ang - s_ang;
        if (e_ang < s_ang) diff += 360.0;
        double angle_step = (lsn > 1) ? (diff / (lsn - 1)) : 0;

        for (int i = 0; i < lsn; i++) {
            uint16_t raw_dis = g_recvbuf[10 + i*3 + 2] << 8 | g_recvbuf[10 + i*3 + 1];
            float dist_m = (float)(((raw_dis >> 8) << 6) | ((raw_dis & 0x00FF) >> 2)) / 1000.0f;
            float intensity = (float)g_recvbuf[10 + i*3];
            
            // 1. 각도 보간 및 앞뒤 반전 (+180)
            double current_angle = s_ang + (angle_step * i) + 180.0;

            // 2. 각도 정규화 (0~360)
            while (current_angle >= 360.0) current_angle -= 360.0;
            while (current_angle < 0.0) current_angle += 360.0;

            // 3. 좌우 반전 (Mirroring) 및 인덱스 매핑
            int angle_idx = (360 - (int)current_angle) % 360;
            
            if (dist_m > 0.05f) {
                msg.ranges[angle_idx] = dist_m;
                msg.intensities[angle_idx] = intensity;
                // 전역 배열 업데이트 (연속성 유지용)
                ranges[angle_idx] = dist_m;
                intensities[angle_idx] = intensity;
            }
        }

        // 전체 스캔 데이터를 담아 발행
        msg.ranges = ranges;
        msg.intensities = intensities;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> ranges;
    std::vector<float> intensities;
    uint8_t g_recvbuf[256];
    int fd, step = 0;
    uint16_t si_len = 0, si_index = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TMiniLidarNode>());
    rclcpp::shutdown();
    return 0;
}
