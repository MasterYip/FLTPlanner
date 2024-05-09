/**
 * @file benchmark.hpp
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
#include <iostream>
#include <vector>
/* external project header files */

/* internal project header files */
#include "timer.hpp"

enum RecordType
{
    NORMAL,
    CRITICAL,
    MISC
};

struct Record
{
    uint type;
    std::string name;
    std::string color;
    double time_record;

    Record(double time_record, std::string name = "record",
           uint type = RecordType::NORMAL) : type(type), name(name), time_record(time_record)
    {
        switch (type)
        {
        case RecordType::NORMAL:
            color = "\033[0m";
            break;
        case RecordType::CRITICAL: // green
            color = "\033[1;32m";
            break;
        case RecordType::MISC: // grey
            color = "\033[1;30m";
            break;
        default:
            color = "\033[0m";
            break;
        }
    }

    void print()
    {
        std::cout << color << name << " time:\t" << time_record << " ms\033[0m" << std::endl;
    }
};

struct BenchmarkResult
{
    double normal_tot_time;
    double critic_tot_time;
    double misc_tot_time;
    double tot_time;

    std::vector<double> custom_data;
};

class Benchmark
{
private:
    bool enabled_;
    std::string name_;
    Timer timer_;
    std::vector<Record> records_;
    BenchmarkResult result_;

public:
    Benchmark(std::string name = "Benchmark", bool enabled = true);
    ~Benchmark();
    void reset();
    void record(std::string name, uint type = RecordType::NORMAL);
    void end();

    void addCustomData(double data)
    {
        if (!enabled_)
            return;
        result_.custom_data.emplace_back(data);
    }

    BenchmarkResult getResult() const
    {
        return result_;
    }

    std::vector<Record> getRecords() const
    {
        return records_;
    }
};