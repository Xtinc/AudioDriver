#include "audio_frame.h"
#include <cstring>

// 静态成员初始化
AudioFrame *AudioFrame::freelist = nullptr;
std::mutex AudioFrame::freelist_mutex;

AudioFrame::AudioFrame(uint32_t capacity)
    : sender_id(0), channels(0), frames(0), sample_rate(0), sequence(0), timestamp(0), source_ip(0), pcm_data(nullptr),
      data_capacity(0), data_size(0), next(nullptr)
{
    // 仅在指定了容量时才分配内存
    if (capacity > 0)
    {
        pcm_data = new int16_t[capacity];
        data_capacity = capacity;
    }
}

AudioFrame::~AudioFrame()
{
    if (pcm_data)
    {
        delete[] pcm_data;
        pcm_data = nullptr;
    }
}

bool AudioFrame::check_capacity(uint32_t required_samples)
{
    return data_capacity >= required_samples;
}

AudioFrame *AudioFrame::alloc(uint32_t required_samples)
{
    std::lock_guard<std::mutex> lock(freelist_mutex);

    AudioFrame *frame = nullptr;

    // 尝试从freelist中找到容量满足要求的帧
    if (freelist)
    {
        AudioFrame *prev = nullptr;
        AudioFrame *current = freelist;

        // 遍历freelist寻找合适的帧
        while (current)
        {
            if (current->check_capacity(required_samples))
            {
                // 找到满足容量要求的帧
                if (prev)
                {
                    prev->next = current->next; // 从freelist中移除
                }
                else
                {
                    freelist = current->next; // 是第一个节点
                }
                frame = current;
                frame->next = nullptr;
                break;
            }
            prev = current;
            current = current->next;
        }
    }

    // 如果没找到合适的帧，创建新帧
    if (!frame)
    {
        frame = new AudioFrame(required_samples);
    }

    // 重置帧状态，但保留pcm_data
    frame->reset();

    return frame;
}

void AudioFrame::free(AudioFrame *frame)
{
    if (!frame)
        return;

    // 不释放PCM数据，保留以便复用
    // 只重置帧属性
    frame->reset();

    // 将帧放回freelist
    std::lock_guard<std::mutex> lock(freelist_mutex);
    frame->next = freelist;
    freelist = frame;
}

void AudioFrame::reset()
{
    sender_id = 0;
    channels = 0;
    frames = 0;
    sample_rate = 0;
    sequence = 0;
    timestamp = 0;
    source_ip = 0;
    data_size = 0;
    // pcm_data 和 data_capacity 保持不变以便复用
}

// AudioFrameQueue 实现
AudioFrameQueue::AudioFrameQueue() : head(nullptr), tail(nullptr), count(0)
{
}

AudioFrameQueue::~AudioFrameQueue()
{
    clear();
}

void AudioFrameQueue::insert(AudioFrame *frame)
{
    if (!frame)
        return;

    std::lock_guard<std::mutex> lock(queue_mutex);

    if (!head)
    {
        // 队列为空
        head = tail = frame;
        frame->next = nullptr;
        count = 1;
        return;
    }

    // 按时间戳排序插入
    if (frame->timestamp < head->timestamp)
    {
        // 插入队列头部
        frame->next = head;
        head = frame;
    }
    else if (frame->timestamp >= tail->timestamp)
    {
        // 插入队列尾部
        tail->next = frame;
        frame->next = nullptr;
        tail = frame;
    }
    else
    {
        // 插入队列中间
        AudioFrame *current = head;
        while (current->next && current->next->timestamp <= frame->timestamp)
        {
            current = current->next;
        }
        frame->next = current->next;
        current->next = frame;
    }

    count++;
}

AudioFrame *AudioFrameQueue::pop_front()
{
    std::lock_guard<std::mutex> lock(queue_mutex);

    if (!head)
        return nullptr;

    AudioFrame *frame = head;
    head = head->next;

    if (!head)
        tail = nullptr;

    frame->next = nullptr;
    count--;

    return frame;
}

size_t AudioFrameQueue::size() const
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    return count;
}

void AudioFrameQueue::clear()
{
    std::lock_guard<std::mutex> lock(queue_mutex);

    while (head)
    {
        AudioFrame *temp = head;
        head = head->next;
        AudioFrame::free(temp);
    }

    tail = nullptr;
    count = 0;
}