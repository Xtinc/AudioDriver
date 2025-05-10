#ifndef AUDIO_PROCESSING_ARRAY_VIEW_H
#define AUDIO_PROCESSING_ARRAY_VIEW_H

/**
 * @file audio_dataview.h
 * @brief This file contains 3 types of view classes:
 * MonoView<>: A single channel contiguous buffer of samples.
 * InterleavedView<>: Channel samples are interleaved (side-by-side) in
 * the buffer. A single channel InterleavedView<> is the same thing as a
 * MonoView<>
 * DeinterleavedView<>: Each channel's samples are contiguous within the
 * buffer. Channels can be enumerated and accessing the individual channel
 * data is done via MonoView<>.
 * The views are comparable to and built on webrtc::ArrayView<> but add
 * audio specific properties for the dimensions of the buffer and the above
 * specialized [de]interleaved support.
 * There are also a few generic utility functions that can simplify
 * generic code for supporting more than one type of view.
 */

#include "audio_message.h"
#include <algorithm>
#include <array>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

namespace array_view_internal
{
/**
 * @brief Determines if the given class has zero-argument .data() and .size() methods
 * whose return values are convertible to T* and size_t, respectively.
 *
 * @tparam DS The data structure type to check
 * @tparam T The pointer type that DS::data() should convert to
 */
template <typename DS, typename T> class HasDataAndSize
{
  private:
    template <typename C,
              std::enable_if_t<std::is_convertible<decltype(std::declval<C>().data()), T *>::value &&
                               std::is_convertible<decltype(std::declval<C>().size()), std::size_t>::value> * = nullptr>
    static int Test(int);

    template <typename> static char Test(...);

  public:
    static constexpr bool value = std::is_same<decltype(Test<DS>(0)), int>::value;
};

/**
 * @brief Magic constant for indicating that the size of an ArrayView is variable
 */
enum : std::ptrdiff_t
{
    kArrayViewVarSize = -4711
};

/**
 * @brief Base class for ArrayViews of fixed nonzero size.
 *
 * @tparam T The element type of the array view
 * @tparam Size The fixed size of the array view
 */
template <typename T, std::ptrdiff_t Size> class ArrayViewBase
{
    static_assert(Size > 0, "ArrayView size must be variable or non-negative");

  public:
    ArrayViewBase(T *data, size_t /* size */) : data_(data)
    {
    }

    static constexpr size_t size()
    {
        return Size;
    }

    static constexpr bool empty()
    {
        return false;
    }

    T *data() const
    {
        return data_;
    }

  protected:
    static constexpr bool fixed_size()
    {
        return true;
    }

  private:
    T *data_;
};

/**
 * @brief Specialized base class for ArrayViews of fixed zero size.
 *
 * @tparam T The element type of the array view
 */
template <typename T> class ArrayViewBase<T, 0>
{
  public:
    explicit ArrayViewBase(T * /* data */, size_t /* size */)
    {
    }

    static constexpr size_t size()
    {
        return 0;
    }

    static constexpr bool empty()
    {
        return true;
    }

    T *data() const
    {
        return nullptr;
    }

  protected:
    static constexpr bool fixed_size()
    {
        return true;
    }
};

/**
 * @brief Specialized base class for ArrayViews of variable size.
 *
 * @tparam T The element type of the array view
 */
template <typename T> class ArrayViewBase<T, kArrayViewVarSize>
{
  public:
    ArrayViewBase(T *data, size_t size) : data_(size == 0 ? nullptr : data), size_(size)
    {
    }

    size_t size() const
    {
        return size_;
    }

    bool empty() const
    {
        return size_ == 0;
    }

    T *data() const
    {
        return data_;
    }

  protected:
    static constexpr bool fixed_size()
    {
        return false;
    }

  private:
    T *data_;
    size_t size_;
};

} // namespace array_view_internal

/**
 * @brief ArrayView is the same thing as gsl::span from the Guideline Support Library.
 *
 * Many functions read from or write to arrays. The obvious way to do this is
 * to use two arguments, a pointer to the first element and an element count:
 *
 * @code{.cpp}
 *   bool Contains17(const int* arr, size_t size) {
 *     for (size_t i = 0; i < size; ++i) {
 *       if (arr[i] == 17)
 *         return true;
 *     }
 *     return false;
 *   }
 * @endcode
 *
 * This is flexible, since it doesn't matter how the array is stored (C array,
 * std::vector, Buffer, ...), but it's error-prone because the caller
 * has to correctly specify the array length:
 *
 * @code{.cpp}
 *   Contains17(arr, array size(arr));    // C array
 *   Contains17(arr.data(), arr.size());  // std::vector
 *   Contains17(arr, size);               // pointer + size
 *   ...
 * @endcode
 *
 * It's also kind of messy to have two separate arguments for what is
 * conceptually a single thing.
 *
 * Enter ArrayView<T>. It contains a T pointer (to an array it doesn't
 * own) and a count, and supports the basic things you'd expect, such as
 * indexing and iteration. It allows us to write our function like this:
 *
 * @code{.cpp}
 *   bool Contains17(ArrayView<const int> arr) {
 *     for (auto e : arr) {
 *       if (e == 17)
 *         return true;
 *     }
 *     return false;
 *   }
 * @endcode
 *
 * And even better, because a bunch of things will implicitly convert to
 * ArrayView, we can call it like this:
 *
 * @code{.cpp}
 *   Contains17(arr);                             // C array
 *   Contains17(arr);                             // std::vector
 *   Contains17(ArrayView<int>(arr, size));  // pointer + size
 *   Contains17(nullptr);                         // nullptr -> empty ArrayView
 *   ...
 * @endcode
 *
 * ArrayView<T> stores both a pointer and a size, but you may also use
 * ArrayView<T, N>, which has a size that's fixed at compile time (which means
 * it only has to store the pointer).
 *
 * One important point is that ArrayView<T> and ArrayView<const T> are
 * different types, which allow and don't allow mutation of the array elements,
 * respectively. The implicit conversions work just like you'd hope, so that
 * e.g. vector<int> will convert to either ArrayView<int> or ArrayView<const
 * int>, but const vector<int> will convert only to ArrayView<const int>.
 * (ArrayView itself can be the source type in such conversions, so
 * ArrayView<int> will convert to ArrayView<const int>.)
 *
 * @note ArrayView is tiny (just a pointer and a count if variable-sized, just
 * a pointer if fix-sized) and trivially copyable, so it's probably cheaper to
 * pass it by value than by const reference.
 */
template <typename T, std::ptrdiff_t Size = array_view_internal::kArrayViewVarSize>
class ArrayView final : public array_view_internal::ArrayViewBase<T, Size>
{
  public:
    /** @brief Type of elements stored in the array view */
    using value_type = T;
    /** @brief Reference to an element */
    using reference = value_type &;
    /** @brief Const reference to an element */
    using const_reference = const value_type &;
    /** @brief Pointer to an element */
    using pointer = value_type *;
    /** @brief Const pointer to an element */
    using const_pointer = const value_type *;
    /** @brief Const iterator type */
    using const_iterator = const T *;

    /**
     * @brief Construct an ArrayView from a pointer and a length.
     *
     * @tparam U Type of data pointer
     * @param data Pointer to the first element
     * @param size Number of elements in the array
     */
    template <typename U>
    ArrayView(U *data, size_t size) : array_view_internal::ArrayViewBase<T, Size>::ArrayViewBase(data, size)
    {
    }

    ArrayView() : ArrayView(nullptr, 0)
    {
    }

    ArrayView(std::nullptr_t) // NOLINT
        : ArrayView()
    {
    }

    ArrayView(std::nullptr_t, size_t size) : ArrayView(static_cast<T *>(nullptr), size)
    {
        static_assert(Size == 0 || Size == array_view_internal::kArrayViewVarSize, "");
    }

    /**
     * @brief Construct an ArrayView from a C-style array.
     *
     * @tparam U Element type of the array
     * @tparam N Size of the array
     * @param array Reference to the C-style array
     */
    template <typename U, size_t N> ArrayView(U (&array)[N]) : ArrayView(array, N)
    {
        static_assert(Size == N || Size == array_view_internal::kArrayViewVarSize,
                      "Array size must match ArrayView size");
    }

    /**
     * @brief Construct a fixed size ArrayView<T, N> from a non-const std::array instance.
     *
     * Only available if size is fixed. For an ArrayView with variable size,
     * the used constructor is ArrayView(U& u) instead.
     *
     * @tparam U Element type of the array
     * @tparam N Size of the array
     * @param u Reference to std::array
     */
    template <typename U, size_t N, std::enable_if_t<Size == static_cast<std::ptrdiff_t>(N)> * = nullptr>
    ArrayView(std::array<U, N> &u) : ArrayView(u.data(), u.size())
    {
    }

    /**
     * @brief Construct a fixed size ArrayView<T, N> from a const std::array instance.
     *
     * Only available if size is fixed and T is const. For an ArrayView with
     * variable size, the used constructor is ArrayView(const U& u) instead.
     *
     * @tparam U Element type of the array
     * @tparam N Size of the array
     * @param u Const reference to std::array
     */
    template <typename U, size_t N, std::enable_if_t<Size == static_cast<std::ptrdiff_t>(N)> * = nullptr>
    ArrayView(const std::array<U, N> &u) : ArrayView(u.data(), u.size())
    {
    }

    /**
     * @brief Construct an ArrayView from any type with fixed size.
     *
     * Only available if ArrayView size is fixed. Constructs from any type U that has a
     * static constexpr size() method whose return value is equal to Size, and a
     * data() method whose return value converts implicitly to T*.
     *
     * This allows conversion from ArrayView<T, N> to ArrayView<const T, N>,
     * but not the other way around. Also doesn't allow conversion from
     * ArrayView<T> to ArrayView<T, N>, or from ArrayView<T, M> to ArrayView<T, N>
     * when M != N.
     *
     * @tparam U Source container type
     * @param u Reference to container
     */
    template <typename U, std::enable_if_t<Size != array_view_internal::kArrayViewVarSize &&
                                           array_view_internal::HasDataAndSize<U, T>::value> * = nullptr>
    ArrayView(U &u) : ArrayView(u.data(), u.size())
    {
        static_assert(U::size() == Size, "Sizes must match exactly");
    }

    /**
     * @brief Construct an ArrayView from any const type with fixed size.
     *
     * Only available if ArrayView size is fixed. Constructs from any type U that has a
     * static constexpr size() method whose return value is equal to Size, and a
     * data() method whose return value converts implicitly to T*.
     *
     * @tparam U Source container type
     * @param u Const reference to container
     */
    template <typename U, std::enable_if_t<Size != array_view_internal::kArrayViewVarSize &&
                                           array_view_internal::HasDataAndSize<U, T>::value> * = nullptr>
    ArrayView(const U &u) : ArrayView(u.data(), u.size())
    {
        static_assert(U::size() == Size, "Sizes must match exactly");
    }

    /**
     * @brief Construct an ArrayView from any type with variable size.
     *
     * Only available if size is variable. Constructs from any type U that has a
     * size() method whose return value converts implicitly to size_t, and a
     * data() method whose return value converts implicitly to T*.
     *
     * This allows conversion from ArrayView<T> to ArrayView<const T>, but not
     * the other way around. Other allowed conversions include:
     * - ArrayView<T, N> to ArrayView<T> or ArrayView<const T>
     * - std::vector<T> to ArrayView<T> or ArrayView<const T>
     * - const std::vector<T> to ArrayView<const T>
     * - Buffer to ArrayView<uint8_t> or ArrayView<const uint8_t>
     * - const Buffer to ArrayView<const uint8_t>
     *
     * @tparam U Source container type
     * @param u Reference to container
     */
    template <typename U, std::enable_if_t<Size == array_view_internal::kArrayViewVarSize &&
                                           array_view_internal::HasDataAndSize<U, T>::value> * = nullptr>
    ArrayView(U &u) : ArrayView(u.data(), u.size())
    {
    }

    /**
     * @brief Construct an ArrayView from any const type with variable size.
     *
     * Only available if size is variable. Constructs from any const type U that has a
     * size() method whose return value converts implicitly to size_t, and a
     * data() method whose return value converts implicitly to T*.
     *
     * @tparam U Source container type
     * @param u Const reference to container
     */
    template <typename U, std::enable_if_t<Size == array_view_internal::kArrayViewVarSize &&
                                           array_view_internal::HasDataAndSize<U, T>::value> * = nullptr>
    ArrayView(const U &u) : ArrayView(u.data(), u.size())
    {
    }

    T &operator[](size_t idx) const
    {
        return this->data()[idx];
    }

    T *begin() const
    {
        return this->data();
    }

    T *end() const
    {
        return this->data() + this->size();
    }

    const T *cbegin() const
    {
        return this->data();
    }

    const T *cend() const
    {
        return this->data() + this->size();
    }

    std::reverse_iterator<T *> rbegin() const
    {
        return std::make_reverse_iterator(end());
    }

    std::reverse_iterator<T *> rend() const
    {
        return std::make_reverse_iterator(begin());
    }

    std::reverse_iterator<const T *> crbegin() const
    {
        return std::make_reverse_iterator(cend());
    }

    std::reverse_iterator<const T *> crend() const
    {
        return std::make_reverse_iterator(cbegin());
    }

    /**
     * @brief Create a subview from this ArrayView with specified offset and size
     *
     * @param offset Start position for the subview
     * @param size Maximum size of the subview
     * @return ArrayView<T> A new ArrayView representing the subview
     */
    ArrayView<T> subview(size_t offset, size_t size) const
    {
        return offset < this->size() ? ArrayView<T>(this->data() + offset, std::min(size, this->size() - offset))
                                     : ArrayView<T>();
    }

    /**
     * @brief Create a subview from this ArrayView with specified offset
     *
     * @param offset Start position for the subview
     * @return ArrayView<T> A new ArrayView representing the subview
     */
    ArrayView<T> subview(size_t offset) const
    {
        return subview(offset, this->size());
    }
};

/**
 * @brief Compare two ArrayViews for equality.
 *
 * Compares their (pointer,size) pairs; it does *not* dereference the pointers.
 *
 * @tparam T The element type
 * @tparam Size1 Size of the first ArrayView
 * @tparam Size2 Size of the second ArrayView
 * @param a First ArrayView to compare
 * @param b Second ArrayView to compare
 * @return true if both ArrayViews point to the same memory region
 */
template <typename T, std::ptrdiff_t Size1, std::ptrdiff_t Size2>
bool operator==(const ArrayView<T, Size1> &a, const ArrayView<T, Size2> &b)
{
    return a.data() == b.data() && a.size() == b.size();
}

/**
 * @brief Compare two ArrayViews for inequality.
 *
 * @tparam T The element type
 * @tparam Size1 Size of the first ArrayView
 * @tparam Size2 Size of the second ArrayView
 * @param a First ArrayView to compare
 * @param b Second ArrayView to compare
 * @return true if the ArrayViews do not point to the same memory region
 */
template <typename T, std::ptrdiff_t Size1, std::ptrdiff_t Size2>
bool operator!=(const ArrayView<T, Size1> &a, const ArrayView<T, Size2> &b)
{
    return !(a == b);
}

// Variable-size ArrayViews are the size of two pointers; fixed-size ArrayViews
// are the size of one pointer. (And as a special case, fixed-size ArrayViews
// of size 0 require no storage.)
static_assert(sizeof(ArrayView<int>) == 2 * sizeof(int *), "");
static_assert(sizeof(ArrayView<int, 17>) == sizeof(int *), "");
static_assert(std::is_empty<ArrayView<int, 0>>::value, "");

/**
 * @brief Helper function to create an ArrayView from a pointer and size
 *
 * @tparam T Element type of the array
 * @param data Pointer to the first element
 * @param size Number of elements
 * @return ArrayView<T> The created array view
 */
template <typename T> ArrayView<T> MakeArrayView(T *data, size_t size)
{
    return ArrayView<T>(data, size);
}

/**
 * @brief Reinterpret an ArrayView as a view of another primitive type with same size and alignment.
 *
 * Only for primitive types that have the same size and alignment.
 * Allows to reinterpret cast of the array view to another primitive type of the same size.
 * Template arguments order is (U, T, Size) to allow deduction of the template
 * arguments in client calls: reinterpret_array_view<target_type>(array_view).
 *
 * @tparam U Target primitive type
 * @tparam T Source primitive type
 * @tparam Size Size of the array view
 * @param view Source array view to reinterpret
 * @return ArrayView<U, Size> Reinterpreted array view
 */
template <typename U, typename T, std::ptrdiff_t Size>
ArrayView<U, Size> reinterpret_array_view(ArrayView<T, Size> view)
{
    static_assert(sizeof(U) == sizeof(T) && alignof(U) == alignof(T),
                  "ArrayView reinterpret_cast is only supported for casting "
                  "between views that represent the same chunk of memory.");
    static_assert(std::is_fundamental<T>::value && std::is_fundamental<U>::value,
                  "ArrayView reinterpret_cast is only supported for casting between "
                  "fundamental types.");
    return ArrayView<U, Size>(reinterpret_cast<U *>(view.data()), view.size());
}

/**
 * @brief A view over a single contiguous audio buffer
 *
 * MonoView represents a view over a single contiguous audio buffer. This
 * can be either a single channel (mono) interleaved buffer (e.g. AudioFrame),
 * or a de-interleaved channel (e.g. from AudioBuffer).
 *
 * @tparam T Type of the audio samples
 */
template <typename T> using MonoView = ArrayView<T>;

/**
 * @brief A view over an interleaved audio buffer
 *
 * InterleavedView represents a view over an interleaved audio buffer where
 * channel samples are interleaved (side-by-side) in the buffer, such as
 * those from AudioFrame.
 *
 * @tparam T Type of the audio samples
 */
template <typename T> class InterleavedView
{
  public:
    using value_type = T;

    InterleavedView() = default;

    /**
     * @brief Constructs an interleaved view from raw audio data
     *
     * @tparam U Type of the source data pointer
     * @param data Pointer to the interleaved audio data
     * @param samples_per_channel Number of samples in each channel
     * @param num_channels Number of audio channels
     */
    template <typename U>
    InterleavedView(U *data, size_t samples_per_channel, size_t num_channels)
        : num_channels_(num_channels), samples_per_channel_(samples_per_channel),
          data_(data, num_channels * samples_per_channel)
    {
        DBG_ASSERT_COND(num_channels_ == 0u || samples_per_channel_ != 0u);
    }

    /**
     * @brief Construct an InterleavedView from a C-style array
     *
     * Samples per channels is calculated based on the array size / num_channels.
     *
     * @tparam U Type of the array elements
     * @tparam N Size of the array
     * @param array Reference to the C-style array
     * @param num_channels Number of audio channels
     */
    template <typename U, size_t N>
    InterleavedView(U (&array)[N], size_t num_channels) : InterleavedView(array, N / num_channels, num_channels)
    {
        DBG_ASSERT_EQ(N % num_channels, 0u);
    }

    /**
     * @brief Copy constructor from another InterleavedView
     *
     * @tparam U Type of the source view elements
     * @param other Source InterleavedView to copy from
     */
    template <typename U>
    InterleavedView(const InterleavedView<U> &other)
        : num_channels_(other.num_channels()), samples_per_channel_(other.samples_per_channel()), data_(other.data())
    {
    }

    size_t num_channels() const
    {
        return num_channels_;
    }

    size_t samples_per_channel() const
    {
        return samples_per_channel_;
    }

    ArrayView<T> data() const
    {
        return data_;
    }

    bool empty() const
    {
        return data_.empty();
    }

    size_t size() const
    {
        return data_.size();
    }

    /**
     * @brief Converts to a mono view if the data is single channel
     * @return MonoView<T> Mono view of the audio data
     */
    MonoView<T> AsMono() const
    {
        DBG_ASSERT_EQ(num_channels(), 1u);
        DBG_ASSERT_EQ(data_.size(), samples_per_channel_);
        return data_;
    }

    /**
     * @brief Copy data from another InterleavedView into this view
     *
     * A simple wrapper around memcpy that includes checks for properties.
     *
     * @tparam U Type of the source view elements
     * @param source Source InterleavedView to copy from
     */
    template <typename U> void CopyFrom(const InterleavedView<U> &source)
    {
        static_assert(sizeof(T) == sizeof(U), "");
        DBG_ASSERT_EQ(num_channels(), source.num_channels());
        DBG_ASSERT_EQ(samples_per_channel(), source.samples_per_channel());
        DBG_ASSERT_GE(data_.size(), source.data().size());
        const auto data = source.data();
        memcpy(&data_[0], &data[0], data.size() * sizeof(U));
    }

    T &operator[](size_t idx) const
    {
        return data_[idx];
    }

    T *begin() const
    {
        return data_.begin();
    }

    T *end() const
    {
        return data_.end();
    }

    const T *cbegin() const
    {
        return data_.cbegin();
    }

    const T *cend() const
    {
        return data_.cend();
    }

    std::reverse_iterator<T *> rbegin() const
    {
        return data_.rbegin();
    }

    std::reverse_iterator<T *> rend() const
    {
        return data_.rend();
    }

    std::reverse_iterator<const T *> crbegin() const
    {
        return data_.crbegin();
    }

    std::reverse_iterator<const T *> crend() const
    {
        return data_.crend();
    }

  private:
    // TODO(tommi): Consider having these both be stored as uint16_t to
    // save a few bytes per view. Use `dchecked_cast` to support size_t during
    // construction.
    size_t num_channels_ = 0u;
    size_t samples_per_channel_ = 0u;
    ArrayView<T> data_;
};

/**
 * @brief A view over a deinterleaved audio buffer
 *
 * DeinterleavedView represents a view over a deinterleaved audio buffer where
 * each channel's samples are contiguous within the buffer. Channels can be
 * enumerated and accessed individually as MonoView objects.
 *
 * @tparam T Type of the audio samples
 */
template <typename T> class DeinterleavedView
{
  public:
    using value_type = T;

    DeinterleavedView() = default;

    /**
     * @brief Constructs a deinterleaved view from raw audio data
     *
     * @tparam U Type of the source data pointer
     * @param data Pointer to the deinterleaved audio data
     * @param samples_per_channel Number of samples in each channel
     * @param num_channels Number of audio channels
     */
    template <typename U>
    DeinterleavedView(U *data, size_t samples_per_channel, size_t num_channels)
        : num_channels_(num_channels), samples_per_channel_(samples_per_channel),
          data_(data, num_channels * samples_per_channel_)
    {
    }

    /**
     * @brief Copy constructor from another DeinterleavedView
     *
     * @tparam U Type of the source view elements
     * @param other Source DeinterleavedView to copy from
     */
    template <typename U>
    DeinterleavedView(const DeinterleavedView<U> &other)
        : num_channels_(other.num_channels()), samples_per_channel_(other.samples_per_channel()), data_(other.data())
    {
    }

    /**
     * @brief Returns a view of a specific channel
     *
     * Returns a deinterleaved channel where `idx` is the zero based index,
     * in the range [0 ... num_channels()-1].
     *
     * @param idx Zero-based channel index
     * @return MonoView<T> View of the specified channel
     */
    MonoView<T> operator[](size_t idx) const
    {
        DBG_ASSERT_LT(idx, num_channels_);
        return MonoView<T>(&data_[idx * samples_per_channel_], samples_per_channel_);
    }

    size_t num_channels() const
    {
        return num_channels_;
    }

    size_t samples_per_channel() const
    {
        return samples_per_channel_;
    }

    ArrayView<T> data() const
    {
        return data_;
    }

    bool empty() const
    {
        return data_.empty();
    }

    size_t size() const
    {
        return data_.size();
    }

    /**
     * @brief Returns the first (and possibly only) channel
     * @return MonoView<T> View of the first channel
     */
    MonoView<T> AsMono() const
    {
        DBG_ASSERT_GE(num_channels(), 1u);
        return (*this)[0];
    }

  private:
    // TODO(tommi): Consider having these be stored as uint16_t to save a few
    // bytes per view. Use `dchecked_cast` to support size_t during construction.
    size_t num_channels_ = 0u;
    size_t samples_per_channel_ = 0u;
    ArrayView<T> data_;
};

template <typename T> constexpr size_t NumChannels(const MonoView<T> & /* view */)
{
    return 1u;
}

template <typename T> size_t NumChannels(const InterleavedView<T> &view)
{
    return view.num_channels();
}

template <typename T> size_t NumChannels(const DeinterleavedView<T> &view)
{
    return view.num_channels();
}

template <typename T> constexpr bool IsMono(const MonoView<T> & /* view */)
{
    return true;
}

template <typename T> constexpr bool IsInterleavedView(const MonoView<T> & /* view */)
{
    return true;
}

template <typename T> constexpr bool IsInterleavedView(const InterleavedView<T> & /* view */)
{
    return true;
}

template <typename T> constexpr bool IsInterleavedView(const DeinterleavedView<const T> & /* view */)
{
    return false;
}

template <typename T> bool IsMono(const InterleavedView<T> &view)
{
    return NumChannels(view) == 1u;
}

template <typename T> bool IsMono(const DeinterleavedView<T> &view)
{
    return NumChannels(view) == 1u;
}

template <typename T> size_t SamplesPerChannel(const MonoView<T> &view)
{
    return view.size();
}

template <typename T> size_t SamplesPerChannel(const InterleavedView<T> &view)
{
    return view.samples_per_channel();
}

template <typename T> size_t SamplesPerChannel(const DeinterleavedView<T> &view)
{
    return view.samples_per_channel();
}
/**
 * @brief Copy samples from one audio view to another
 *
 * A simple wrapper around memcpy that includes checks for properties.
 * The parameter order is the same as for memcpy(), first destination then
 * source.
 *
 * @tparam D Destination view type
 * @tparam S Source view type
 * @param destination Destination audio view
 * @param source Source audio view
 */
template <typename D, typename S> void CopySamples(D &destination, const S &source)
{
    static_assert(sizeof(typename D::value_type) == sizeof(typename S::value_type), "");
    // Here we'd really like to do
    // static_assert(IsInterleavedView(destination) == IsInterleavedView(source),
    //               "");
    // but the compiler doesn't like it inside this template function for
    // some reason. The following check is an approximation but unfortunately
    // means that copying between a MonoView and single channel interleaved or
    // deinterleaved views wouldn't work.
    // static_assert(sizeof(destination) == sizeof(source),
    //               "Incompatible view types");
    DBG_ASSERT_EQ(NumChannels(destination), NumChannels(source));
    DBG_ASSERT_EQ(SamplesPerChannel(destination), SamplesPerChannel(source));
    DBG_ASSERT_GE(destination.size(), source.size());
    memcpy(&destination[0], &source[0], source.size() * sizeof(typename S::value_type));
}

// Sets all the samples in a view to 0. This template function is a simple
// wrapper around `memset()` but adds the benefit of automatically calculating
// the byte size from the number of samples and sample type.
template <typename T> void ClearSamples(T &view)
{
    memset(&view[0], 0, view.size() * sizeof(typename T::value_type));
}

// Same as `ClearSamples()` above but allows for clearing only the first
// `sample_count` number of samples.
template <typename T> void ClearSamples(T &view, size_t sample_count)
{
    DBG_ASSERT_LE(sample_count, view.size());
    memset(&view[0], 0, sample_count * sizeof(typename T::value_type));
}

#endif
