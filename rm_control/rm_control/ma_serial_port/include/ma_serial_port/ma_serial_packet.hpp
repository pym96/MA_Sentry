#ifndef __MA_SERIAL_PACKET_HPP__
#define __MA_SERIAL_PACKET_HPP__

#include <ros/ros.h>
#include <serial/serial.h>

namespace ma_serial_packet{
    
    class MA_SerialPacket{
        
        private:

            ros::NodeHandle nh_;
            ros::Subscriber sub;
            ros::Publisher pub;
            serial::Serial ser;
            

        
        public:


    };

}

#endif