#ifndef AUDIO_FRAME_HEADER
#define AUDIO_FRAME_HEADER

#include <cstdint>
#include <mutex>

/**
 * @brief Audio frame class representing decoded PCM data
 */
class AudioFrame
{
  public:
    /**
     * @brief Audio data properties
     */
    uint8_t sender_id;
    uint8_t channels;
    uint32_t frames;
    uint32_t sample_rate;
    uint32_t sequence;
    uint64_t timestamp;
    uint32_t source_ip;

    /**
     * @brief PCM audio data buffer
     */
    int16_t *pcm_data;
    uint32_t data_capacity; /**< Buffer capacity (in samples) */
    uint32_t data_size;     /**< Actual data size used (in bytes) */

    /**
     * @brief Linked list pointer and freelist management
     */
    AudioFrame *next;
    static AudioFrame *freelist;
    static std::mutex freelist_mutex;

    /**
     * @brief Static methods for allocating and releasing frames
     * @param required_samples Number of samples required for allocation
     * @return Pointer to allocated AudioFrame
     */
    static AudioFrame *alloc(uint32_t required_samples);

    /**
     * @brief Return a frame to the freelist
     * @param frame Pointer to the frame to be freed
     */
    static void free(AudioFrame *frame);

    /**
     * @brief Reset frame content for reuse
     */
    void reset();

    /**
     * @brief Check if buffer capacity is sufficient
     * @param required_samples Number of samples needed
     * @return true if capacity is sufficient, false otherwise
     */
    bool check_capacity(uint32_t required_samples);

  private:
    /**
     * @brief Private constructor, frames can only be created via alloc
     * @param capacity Initial buffer capacity in samples
     */
    AudioFrame(uint32_t capacity);

    /**
     * @brief Destructor
     */
    ~AudioFrame();
};

/**
 * @brief Ordered audio frame queue, sorted by timestamp
 */
class AudioFrameQueue
{
  public:
    /**
     * @brief Constructor
     */
    AudioFrameQueue();

    /**
     * @brief Destructor
     */
    ~AudioFrameQueue();

    /**
     * @brief Insert frame into queue, sorted by timestamp
     * @param frame Pointer to the frame to be inserted
     */
    void insert(AudioFrame *frame);

    /**
     * @brief Remove and return the frame at the front of the queue
     * @return Pointer to the removed frame
     */
    AudioFrame *pop_front();

    /**
     * @brief Get the current size of the queue
     * @return Number of frames in the queue
     */
    size_t size() const;

    /**
     * @brief Clear all frames from the queue
     */
    void clear();

  private:
    AudioFrame *head;
    AudioFrame *tail;
    size_t count;
    mutable std::mutex queue_mutex;
};

#endif // AUDIO_FRAME_HEADER