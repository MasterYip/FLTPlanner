#ifndef ZOBRIST_H
#define ZOBRIST_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <random>
#include <algorithm>
#include <search_tree.h>
struct Item {
    std::string key;
    TreeNode_ptr value;

    Item(const std::string& key, TreeNode_ptr value) : key(key), value(value) {}
    // 深度拷贝的拷贝构造函数
    Item(const Item& other) : key(other.key) {
        if (other.value) {
            value = std::make_shared<TreeNode>(*other.value);
        } else {
            value = nullptr;
        }
    }

    Item() : key(""), value(nullptr) {}
};

class HashTable {
private:

    int tableSize;
    int entriesCount;
    int alphabetSize;
    std::vector<std::vector<uint64_t>> zobristnum;
    int hash_index_bits;
    int hash_table_max_size;
    int S;  // 最大深度
    int P;  // 字符个数
    std::string val;
    int nprocs;  // 进程数

public:    
    // 将 hashTable 从 std::map 改为了 std::unordered_map，以提高插入和搜索的性能
    // std::unordered_map是C++标准库提供的一个哈希表容器，用于存储键值对。
    // 它提供了快速的查找、插入和删除操作，并且允许通过键快速访问对应的值。
    std::unordered_map<std::string, Item> hashTable;
    HashTable(int nprocs, const std::string& val, int max_len, int val_len);
    std::pair<int, int> hashing(const std::string& board);
    void insert(const Item& item);
    TreeNode_ptr search_table(const std::string& key);
};


#endif






