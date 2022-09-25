#include <iostream>
#include "ros2_olei/input.h"
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <pcl/visualization/cloud_viewer.h>

// ros2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

//#define PCL_VIEW

std::mutex buffer_lock;
std::mutex info_lock;

struct info{
    int packet_size;
    int start_pos;
    int end_pos;
    info(int s, int e, int size):start_pos(s),end_pos(e),packet_size(size){}
};

struct frame_info{
    std::queue<info> frames;
    int frame_size = 0;
}frame_info;

std::queue<std::shared_ptr<ros2_olei::olePacket>> packet_buffer;

float ChList[16] = { -15.0f, 1.0f, -13.0f, 3.0f, -11.0f, 5.0f, -9.0f, 7.0f, -7.0f, 9.0f, -5.0f, 11.0f, -3.0f, 13.0f, -1.0f, 15.0f };
float offsetV[16] = { 5.06f, -9.15f, 5.06f, -9.15f, 5.06f, -9.15f, 5.06f, -9.15f, 9.15f, -5.06f, 9.15f, -5.06f, 9.15f, -5.06f, 9.15f, -5.06f };
float offsetH[16] = { 21.0f, 21.0f, 21.0f, 21.0f, 21.0f, 21.0f, 21.0f, 21.0f, -21.0f, -21.0f, -21.0f, -21.0f, -21.0f, -21.0f, -21.0f, -21.0f };

bool AddPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float r, float ang, uint8_t intensity, uint8_t chIndex)
{
    float _ang = (ang + chIndex * 0.00108f * 10.0f) * 3.14159265358979f / 180.0f;
    float _w = ChList[chIndex] * 3.14159265358979f / 180.0f;

    /*
     名词解释：
        雷达各个通道输出的测量距离，设为R注意雷达输入的单位为2mm，请先换算为1mm）
        雷达转速，                    设为H（一般为10Hz）
        雷达各个通道的垂直角，        设为ω
        雷达输出的水平角度，        设为α
        雷达各个通道的水平偏移量    设为A
        雷达各个通道的垂直偏移量    设为B
        雷达各个通道的空间坐标系    设为X,Y,Z
        水平差补系数表             offsetH
        垂直差补系数表             offsetV
        空间坐标计算公式为：
        X = R * cos(ω) * sin(α) + A * cos(α)
        Y = R * cos(ω) * cos(α) - A * sin(α)
        Z = R * sin(ω) + B
    */
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (r > 0)
    {
        pcl::PointXYZI pt;
        pt.x = (float)(r * cos(_w) * cos(_ang) + offsetH[chIndex] * cos(_ang) * 0.001f);
        pt.y = (float)(r * cos(_w) * sin(_ang) - offsetH[chIndex] * sin(_ang) * 0.001f);
        pt.z = (float)(r * sin(_w) + offsetV[chIndex] * 0.001f);
        pt.intensity = intensity;
        cloud->points.emplace_back(pt);
        return true;
    }
    return false;
}


void read_thread()
{
    ros2_olei::InputSocket socket;
    int packet_num = 0;
    while (true)
    {
        std::shared_ptr<ros2_olei::olePacket> packet(new ros2_olei::olePacket());
        socket.getPacket(packet, 1);

        buffer_lock.lock();
        packet_buffer.push(packet);
        buffer_lock.unlock();
        packet_num ++;

        for (int i = 0;i < 12;i ++)
        {
            static int last_angle = packet->data.packet.block[i].azimuth;
            int angle = packet->data.packet.block[i].azimuth;

            if (angle < last_angle)
            {
                info_lock.lock();
                frame_info.frame_size ++;
                if (frame_info.frames.empty()){
                    info f{0, i, packet_num};
                    frame_info.frames.push(f);
                }
                else {
                    info f{frame_info.frames.back().end_pos, i, packet_num};
                    frame_info.frames.push(f);
                }
                info_lock.unlock();
                packet_num = 1;
            }
            last_angle = angle;
        }
    }
}

int main(int argc, char ** argv)
{
    // ros2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("olei_laser");
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

#ifdef PCL_VIEW
    pcl::visualization::PCLVisualizer view("laser");
    view.setBackgroundColor(1.0, 1.0, 1.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> view_h(cloud, 0, 255, 0);
    view.addPointCloud(cloud, view_h, "show");
#endif // PCL_VIEW

    std::thread t1(read_thread);
//    t1.join();

    while (true)
    {
        if (frame_info.frame_size > 0)
        {
            std::vector<std::shared_ptr<ros2_olei::olePacket>> frame_data;

            info_lock.lock();
            auto f = frame_info.frames.front();
            frame_info.frames.pop();
            frame_info.frame_size --;
            info_lock.unlock();

            buffer_lock.lock();
            for (int i = 0; i < f.packet_size; i++)
            {
                frame_data.emplace_back(packet_buffer.front());
                if (i < f.packet_size - 1)
                {
                    packet_buffer.pop();
                }
            }
            buffer_lock.unlock();

            cloud->points.clear();
            for (int i = 0; i < f.packet_size; i++)
            {
                int s = i == 0 ? f.start_pos : 0;
                int e = i == f.packet_size - 1 ? f.end_pos : 12;
                for (int j = s; j < e; j++)
                {
                    auto& block = frame_data.at(i)->data.packet.block[j];
                    float angle = (float)block.azimuth * 0.01f;

                    for (uint8_t k = 0; k < 16; k++)
                    {
                        AddPoint(cloud, block.points[k].distance, angle, block.points[k].intensity, k);
                    }
                    for (uint8_t k = 0; k < 16; k++)
                    {
                        AddPoint(cloud, block.points[k+16].distance, angle+0.18f, block.points[k+16].intensity, k);
                    }
                }
            }
#ifdef PCL_VIEW
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> field(cloud, "intensity");
            view.updatePointCloud(cloud, field, "show");
#endif
            // ros2 publish
            sensor_msgs::msg::PointCloud2 ros2_cloud;
            pcl::toROSMsg(*cloud, ros2_cloud);
            ros2_cloud.header.frame_id = "laser";
            publisher->publish(ros2_cloud);
        }
#ifdef PCL_VIEW
        view.spinOnce();
#endif
        // ros2 spin
        rclcpp::spin_some(node);
    }
}
