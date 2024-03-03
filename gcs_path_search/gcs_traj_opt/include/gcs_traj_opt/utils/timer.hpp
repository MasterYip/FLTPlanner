/**
 * @file timer.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <chrono>
/* external project header files */

/* internal project header files */

class TimerMixin
{
protected:
    timespec ts{};
    std::chrono::time_point<std::chrono::system_clock> time_point_;

public:
    TimerMixin(){};
    virtual ~TimerMixin() = default;
    void nanoSleep(uint64_t ns)
    {
        ts.tv_sec = ns / 1000000000;
        ts.tv_nsec = ns % 1000000000;
        nanosleep(&ts, NULL);
    }
    void milliSleep(uint64_t ms)
    {
        ts.tv_sec = ms / 1000;
        ts.tv_nsec = (ms % 1000) * 1000000;
        nanosleep(&ts, NULL);
    }
    void timerReset(void)
    {
        time_point_ = std::chrono::system_clock::now();
    }

    /**
     * @brief CHeck elapsed time in nanoseconds
     * @note Call timerStart() before calling this function (multiple times if needed)
     *
     * @return uint64_t elapsed time in nanoseconds
     */
    uint64_t timerCheck(void)
    {
        auto time_point_now = std::chrono::system_clock::now();
        auto duration =
            std::chrono::duration_cast<std::chrono::nanoseconds>(time_point_now - time_point_);
        return duration.count();
    }
};
typedef TimerMixin Timer;