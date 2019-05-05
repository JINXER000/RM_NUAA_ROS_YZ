#ifndef TIMER_H
#define TIMER_H
#include <sys/time.h>
#include <stdio.h>
#include <iostream>

struct timer{

    timer(std::string info):
        event_info(info)
    {
        gettimeofday(&t_b, 0);
    }

    double elapsed()
    {
        gettimeofday(&t_e, 0);
        double timefly=(t_e.tv_sec - t_b.tv_sec) + (t_e.tv_usec - t_b.tv_usec)*1e-6;
        std::cout <<"time of "<<event_info<<" is "<<timefly<<std::endl;
        return timefly;
    }
    timeval t_b, t_e;
    std::string event_info;
};
#endif // TIMER_H
