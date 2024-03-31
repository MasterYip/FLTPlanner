#include <saveHashTable.h>

namespace SAVE_HASH_TABLE
{
    
    // 存储函数
    void saveBinaryFile(const std::vector<saveData>& data, const std::string& filename)
    {
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
            return;
        }

        // 存储数据数量
        size_t dataSize = data.size();
        file.write(reinterpret_cast<const char*>(&dataSize), sizeof(size_t));

        // 存储每个 saveData 对象的成员数据
        for (const saveData& item : data)
        {
            // 存储 RobotState
            file.write(reinterpret_cast<const char*>(&item.rState), sizeof(MDT::RobotState));

            // 存储 score
            file.write(reinterpret_cast<const char*>(&item.score), sizeof(double));

            // 存储 simDis
            file.write(reinterpret_cast<const char*>(&item.simDis), sizeof(double));

            // 存储 hashKey
            size_t hashKeySize = item.hashKey.size();
            file.write(reinterpret_cast<const char*>(&hashKeySize), sizeof(size_t));
            file.write(item.hashKey.c_str(), hashKeySize);

            // 存储 parentKey
            size_t parentKeySize = item.parentKey.size();
            file.write(reinterpret_cast<const char*>(&parentKeySize), sizeof(size_t));
            file.write(item.parentKey.c_str(), parentKeySize);

            // 存储 visits
            file.write(reinterpret_cast<const char*>(&item.visits), sizeof(int));

            // 存储 num_thread_visited
            file.write(reinterpret_cast<const char*>(&item.num_thread_visited), sizeof(int));
        }

        file.close();
    }

    // 读取函数
    std::vector<saveData> readBinaryFile(const std::string& filename)
    {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file for reading: " << filename << std::endl;
            return {};
        }

        std::vector<saveData> data;

        // 读取数据数量
        size_t dataSize;
        file.read(reinterpret_cast<char*>(&dataSize), sizeof(size_t));

        // 读取每个 saveData 对象的成员数据
        for (size_t i = 0; i < dataSize; ++i)
        {
            saveData item;

            // 读取 RobotState
            file.read(reinterpret_cast<char*>(&item.rState), sizeof(MDT::RobotState));

            // 读取 score
            file.read(reinterpret_cast<char*>(&item.score), sizeof(double));

            // 读取 simDis
            file.read(reinterpret_cast<char*>(&item.simDis), sizeof(double));

            // 读取 hashKey
            size_t hashKeySize;
            file.read(reinterpret_cast<char*>(&hashKeySize), sizeof(size_t));
            item.hashKey.resize(hashKeySize);
            file.read(&item.hashKey[0], hashKeySize);

            // 读取 parentKey
            size_t parentKeySize;
            file.read(reinterpret_cast<char*>(&parentKeySize), sizeof(size_t));
            item.parentKey.resize(parentKeySize);
            file.read(&item.parentKey[0], parentKeySize);

            // 读取 visits
            file.read(reinterpret_cast<char*>(&item.visits), sizeof(int));

            // 读取 num_thread_visited
            file.read(reinterpret_cast<char*>(&item.num_thread_visited), sizeof(int));

            data.push_back(item);
        }

        file.close();

        return data;
    }

    
}

