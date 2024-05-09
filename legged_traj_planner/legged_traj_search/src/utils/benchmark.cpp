/**
 * @file benchmark.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_search/utils/benchmark.hpp"

Benchmark::Benchmark(std::string name, bool enabled) : name_(name), enabled_(enabled)
{
}

Benchmark::~Benchmark()
{
}

void Benchmark::reset()
{
    if (!enabled_)
        return;
    timer_.timerReset();
    records_.clear();
    std::cout << "========== " << name_ << " ==========" << std::endl;
}

void Benchmark::record(std::string name, uint type)
{
    if (!enabled_)
        return;
    records_.emplace_back(Record((double)timer_.timerCheck() / 1e6, name, type));
    timer_.timerReset();
}

void Benchmark::end()
{
    if (!enabled_)
        return;
    
    for (auto &record : records_)
    {
        record.print();
    }

    // Summary
    std::cout << "\033[1;31m"; // red
    for (auto type : {RecordType::NORMAL, RecordType::CRITICAL, RecordType::MISC})
    {
        double total_time = 0.0;
        for (auto &record : records_)
        {
            if (record.type == type)
            {
                total_time += record.time_record;
            }
        }
        switch (type)
        {
        case RecordType::NORMAL:
            std::cout << "Normal time:\t" << total_time << " ms" << std::endl;
            result_.normal_tot_time = total_time;
            break;
        case RecordType::CRITICAL:
            std::cout << "Critical time:\t" << total_time << " ms" << std::endl;
            result_.critic_tot_time = total_time;
            break;
        case RecordType::MISC:
            std::cout << "Misc time time:\t" << total_time << " ms" << std::endl;
            result_.misc_tot_time = total_time;
            break;
        }
    }
    double total_time = 0.0;
    for (auto &record : records_)
    {
        total_time += record.time_record;
    }
    std::cout << "Total time:\t" << total_time << " ms\033[0m" << std::endl;
    result_.tot_time = total_time;
}