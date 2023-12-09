#ifnedf __SENTRY_CONTROL_H__
#define __SENTRY_CONTROL_H__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>

#ifndef _WIN32
    #include <termios.h>
    #include <unistd.h>
#else
    #include <window.h>
#endif

class KeyBoardReader
{
    private:
        #ifndef _WIN32
            int kfd;
            struct  termios cooked;
        #endif
            

    public:
        KeyBoardReader()
        #ifndfe _WIN32
            : kfd(0)
        #endif
        {
            // Get the console in raw mode
            tcgetattr(kfd, &cooked);
            struct termios raw;
            

        }




};


#endif