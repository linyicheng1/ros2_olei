#include "ros2_olei/input.h"
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <cstring>

namespace ros2_olei
{
    static const size_t packet_size = 1206;

    Input::Input(uint16_t port) : port_(port)
    {
        devip_str_ = "";// device_ip
        gps_time_ = false;// use gps or not
    }

    InputSocket::InputSocket(uint16_t port) : Input(port)
    {
        sockfd_ = -1;
        if (!devip_str_.empty())
            inet_aton(devip_str_.c_str(), &devip_);
        printf("Open UDP socket: port %d" ,port_);
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1)
        {
            printf("socket error !!!");
            return;
        }
        sockaddr_in my_addr;
        memset(&my_addr, 0, sizeof(my_addr));
        my_addr.sin_family = AF_INET;
        my_addr.sin_port = htons(port);
        my_addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sockfd_, (sockaddr* )&my_addr, sizeof(sockaddr)) == -1)
        {
            printf("bind error !!!");
        }
        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
        {
            printf("non-block");
        }
    }

    InputSocket::~InputSocket()
    {
        (void) close(sockfd_);
    }

    int InputSocket::getPacket(std::shared_ptr<olePacket> pkt, const double time_offset)
    {
        auto time1 = std::chrono::system_clock::now();
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;

        static const int POLL_TIMEOUT = 1000;// one second

        sockaddr_in sender_address{};
        socklen_t  sender_address_len = sizeof(sender_address);

        while (true)
        {
            do
            {
                int retval = poll(fds, 1, POLL_TIMEOUT);
                if (retval < 0)
                {
                    if (errno != EINTR) printf("poll() error: %s\n", strerror(errno));
                    return -1;
                }
                if (retval == 0)
                {
                    printf("ole poll() timeout !\n");
                }
                if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||(fds[0].revents & POLLNVAL))
                {
                    printf("poll() report ole error\n");
                }
            } while ((fds[0].revents & POLLIN) == 0);
            uint8_t tmp[1206] = {0};
            ssize_t nbytes = recvfrom(sockfd_, pkt->data.raw, packet_size, 0, (sockaddr *)&sender_address, &sender_address_len);
//            for (int i = 0;i < 12;i ++)
//            {
//                int angle = (int)tmp[100*i+2] | (int)tmp[100*i+3]<<8;
//                printf(" %f ", (float)angle*0.01f);
//            }
//            printf("\n");
            if (nbytes < 0)
            {
                if (errno != EWOULDBLOCK)
                {
                    printf("recvfail\n");
                    return -1;
                }
            }
            else if ((size_t)nbytes == packet_size)
            {
                if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
                    continue;
                else
                    break;
            }
            printf("incomplete ole packet read: %zd bytes \n", nbytes);
            pkt->time = time1;
        }
    }

}