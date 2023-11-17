'''
Author: NUC12 2205929492@qq.com
Date: 2023-11-17 11:49:35
LastEditors: NUC12
LastEditTime: 2023-11-17 15:46:56
FilePath: \\Fast-Legged-Planner-Test\\fast_legged_planner_py\\utils\\data_structure.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8


class CircleQueue(object):
    """环形队列"""

    def __init__(self, size_act=10):
        """Initialize the queue
        :param size_act: actual size of the queue
        """
        self.size = size_act+1
        self.queue = [0 for i in range(self.size)]
        self.rear = 0       # 队尾指针, 指向最后一个元素的下一个位置
        self.front = 0      # 队首指针, 指向第一个元素

    def at(self, index):
        """获取队列中的元素"""
        if self.is_valid(index):
            return self.queue[(self.front + index) % self.size]
        # print("Index Error!")
        return None

    def is_valid(self, index):
        pos = (self.front + index) % self.size
        return not ((self.rear >= self.front and (pos > self.rear or pos < self.front)) or (self.rear < self.front and pos > self.rear and pos < self.front) or (index >= self.size-1 or index < 0))

    def enqueue(self, item):
        """进队"""
        if not self.is_full():
            self.queue[self.rear] = item
            self.rear = (self.rear + 1) % self.size  # 队尾指针前移
            return True
        else:
            print("队列已满！")
            return False

    def dequeue(self):
        """出队"""
        if not self.is_empty():
            self.front = (self.front + 1) % self.size  # 队首指针前移
            return self.queue[self.front]
        else:
            print("队列为空！")
            return None

    def is_empty(self):
        """判断环形队列是否为空"""
        return self.front == self.rear

    def is_full(self):
        """判断环形队列是否已满"""
        return (self.rear + 1) % self.size == self.front

    def get_length(self):
        """获取环形队列长度"""
        return (self.rear - self.front + self.size) % self.size

    # Debug
    def travel(self):
        """遍历队列元素"""
        if not self.is_empty():
            i = self.front
            while (i != self.rear):
                print(self.queue[i], " ", end="")
                i = (i + 1) % self.size
            print()
        else:
            print("队列为空！")


if __name__ == '__main__':
    q = CircleQueue(5)
    for i in range(5):
        q.enqueue(i)
    q.travel()
    q.dequeue()
    q.dequeue()

    q.travel()
    for i in range(2):
        q.enqueue(i+4)
    q.travel()
    for i in range(-1, 6, 1):
        print(q.at(i))
