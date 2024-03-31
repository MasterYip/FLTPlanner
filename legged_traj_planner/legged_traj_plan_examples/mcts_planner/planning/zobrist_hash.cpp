    
    #include "zobrist_hash.h"
    
    
    /**
     * @brief Construct a new Hash Table. 初始化哈希表
     * @param nprocs 进程数
     * @param val 字符串
     * @param max_len 最大深度
     * @param val_len 字符个数（最大宽度）
    */
    HashTable::HashTable(int nprocs, const std::string& val, int max_len, int val_len) :
            hashTable(), hash_index_bits(32), hash_table_max_size(1u << 32), S(max_len),
            P(val_len), val(val), nprocs(nprocs) {
        zobristnum.resize(S, std::vector<uint64_t>(P, 0));  // S为最大深度，　P为字符个数（宽度）

        std::random_device rd;
        std::seed_seq seed{ 2023 }; // 设置固定的种子值为 2023

        std::mt19937_64 generator(seed);
        std::uniform_int_distribution<uint64_t> distribution(0, (1ull << 64) - 1);

        // std::cout << zobristnum[0].size();
        for (int i = 0; i < S; ++i) {
            for (int j = 0; j < P; ++j) {
                zobristnum[i][j] = distribution(generator);
            }
        }
    }


    /**
     * @brief 通过哈希key和哈希表计算对应唯一的哈希值
     * @param board 哈希key
     * @return uint64_t 哈希值
    */
    std::pair<int, int> HashTable::hashing(const std::string& board) {
        uint64_t hashing_value = 0;
        for (int i = 0; i < board.length(); ++i) {
            int piece = -1;
            if (i < board.length()) {
                auto it = std::find(val.begin(), val.end(), board[i]);
                // 在C++中，当使用std::find函数进行查找时，如果查找失败，返回的迭代器将等于容器的end()迭代器，表示未找到目标元素。
                if (it != val.end()) {
                    piece = std::distance(val.begin(), it);
                }
            }

            if (piece != -1) {
                hashing_value ^= zobristnum[i][piece];
            }else
            {
                // 将board[i]以字符串形式输出
                std::cout << std::string(1, board[i]) << "  is not in the val list" << std::endl;
                exit(1);
            }

        }

        int core_dest = (hashing_value >> this->hash_index_bits) % nprocs;
        return std::make_pair(hashing_value, core_dest);
    }


    /**
     * @brief 向哈希表中插入元素 这段代码实现了一个哈希表的插入操作，如果哈希表中已存在相同的键，则将旧的键值对删除，
     *        然后插入新的键值对。如果哈希表中不存在相同的键，则直接插入新的键值对。
     * @param item 插入元素
    */
    void HashTable::insert(const Item& item) {
        hashTable[item.key] = item;
    }


    /**
     * @brief 从哈希表中查找元素
     * @param key 查找元素的键
     * @return int 查找元素的值
    */
    TreeNode_ptr HashTable::search_table(const std::string& key) {
        if (hashTable.find(key) != hashTable.end()) {
            return hashTable[key].value;
        }
        return nullptr;
    }