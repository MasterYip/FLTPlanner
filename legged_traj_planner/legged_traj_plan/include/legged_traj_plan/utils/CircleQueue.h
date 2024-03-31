#include <iostream>
#include <vector>

template <typename T>
class CircleQueue
{
private:
    int size;
    int rear;
    int front;
    std::vector<T> queue;

public:
    CircleQueue(int queue_size = 100) : size(queue_size + 1), queue(size), rear(0), front(0) {}

    T at(int index)
    {
        if (is_valid(index))
        {
            return queue[(front + index) % size];
        }
        else
        {
            throw std::invalid_argument("Invalid index!");
        }
    }

    bool is_valid(int index)
    {
        int pos = (front + index) % size;
        return !((rear >= front && (pos >= rear || pos < front)) ||
                 (rear < front && pos > rear && pos < front) ||
                 (index >= size - 1 || index < 0));
    }

    bool enqueue(T item)
    {
        if (!is_full())
        {
            queue[rear] = item;
            rear = (rear + 1) % size;
            return true;
        }
        else
        {
            std::cout << "Queue is full!" << std::endl;
            return false;
        }
    }

    T dequeue()
    {
        if (!is_empty())
        {
            front = (front + 1) % size;
            return queue[front];
        }
        else
        {
            throw std::invalid_argument("Queue is empty!");
        }
    }

    bool is_empty()
    {
        return front == rear;
    }

    bool is_full()
    {
        return (rear + 1) % size == front;
    }

    int get_length()
    {
        return (rear - front + size) % size;
    }

    void travel()
    {
        if (!is_empty())
        {
            int i = front;
            while (i != rear)
            {
                std::cout << queue[i] << " ";
                i = (i + 1) % size;
            }
            std::cout << std::endl;
        }
        else
        {
            std::cout << "Queue is empty!" << std::endl;
        }
    }
};
