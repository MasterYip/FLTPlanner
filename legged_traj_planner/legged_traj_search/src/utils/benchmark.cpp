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
#include <fstream>

Benchmark::Benchmark(std::string name, bool enabled) : enabled_(enabled), name_(name)
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
    result_ = BenchmarkResult();
    // std::cout << "========== " << name_ << " ==========" << std::endl;
}

void Benchmark::record(std::string name, uint type)
{
    if (!enabled_)
        return;
    records_.emplace_back(Record((double)timer_.timerCheck() / 1e6, name, type));
    timer_.timerReset();
}

void Benchmark::end(bool summary)
{
    if (!enabled_)
        return;

    for (auto &record : records_)
    {
        printf("%s | ", name_.c_str());
        record.print();
    }

    // Summary
    if (summary)
    {
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
                printf("%s | ", name_.c_str());
                std::cout << "Normal time:\t" << total_time << " ms" << std::endl;
                result_.normal_tot_time = total_time;
                break;
            case RecordType::CRITICAL:
                printf("%s | ", name_.c_str());
                std::cout << "Critical time:\t" << total_time << " ms" << std::endl;
                result_.critic_tot_time = total_time;
                break;
            case RecordType::MISC:
                printf("%s | ", name_.c_str());
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
        printf("%s | ", name_.c_str());
        std::cout << "Total time:\t" << total_time << " ms\033[0m" << std::endl;
        result_.tot_time = total_time;
    }
}

bool Benchmark::save(const std::string &file_path)
{
    if (!enabled_)
        return false;
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Benchmark: Failed to open file " << file_path << std::endl;
        return false;
    }

    file << "Benchmark: " << name_ << std::endl;
    file << "Totaltime: " << result_.tot_time << std::endl;
    file << "Normaltime: " << result_.normal_tot_time << std::endl;
    file << "Criticaltime: " << result_.critic_tot_time << std::endl;
    file << "Misctime: " << result_.misc_tot_time << std::endl;

    file << "CustomData: [";
    for (const auto &data : result_.custom_data)
    {
        file << data << ", ";
    }
    file << "]" << std::endl;

    file << "Records:" << std::endl;
    for (const auto &record : records_)
    {
        file << "  " << record.name << "time: " << record.time_record << std::endl;
    }

    file.close();
    return true;
}