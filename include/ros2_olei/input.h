#ifndef OLEI_INPUT_H
#define OLEI_INPUT_H
#include <memory>
#include <unistd.h>
#include <pcap.h>
#include <netinet/in.h>
#include <string>
#include <chrono>
#include <cstdio>

namespace ros2_olei
{
static uint16_t DATA_PORT_NUMBER = 2368;// default data port
static uint16_t POSITION_PORT_NUMBER = 8308;// default position port


struct DataPoint{
    uint16_t distance : 16;
    uint8_t intensity : 8;
}__attribute__((__packed__));

struct DataBlock{
    uint8_t identifier[2];
    uint16_t azimuth;
    DataPoint points[32];
}__attribute__((__packed__));

struct RawPacket{
    DataBlock block[12];
    uint32_t timestamp;
    uint8_t factory[2];
}__attribute__((__packed__));

typedef struct olePacket
{
    std::chrono::system_clock::time_point time;
    union data{
        RawPacket packet;
        uint8_t raw[1206];
    }data;
}olePacket;

class Input
{
public:
    Input(uint16_t port);
    virtual ~Input() {}
    virtual int getPacket(std::shared_ptr<olePacket> pkg, const double time_offset) = 0;
protected:
    uint16_t port_;
    std::string devip_str_;
    bool gps_time_;
};

class InputSocket: public Input
{
public:
    InputSocket(uint16_t port = DATA_PORT_NUMBER);
    virtual ~InputSocket();

    virtual int getPacket(std::shared_ptr<olePacket> pkt, const double time_offset);

private:
    int sockfd_;
    in_addr devip_;
};
}

#endif //OLEI_INPUT_H
