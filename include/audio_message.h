#ifndef AUDIO_MSG_PRINT_OPTS_H
#define AUDIO_MSG_PRINT_OPTS_H

/**
 * @file msg_print_opts.h
 * @brief Message printing options for the AudioDriver library
 *
 * This header defines macros for printing messages with different severity levels
 * (info, error, debug) in the AudioDriver library. These macros can be used to
 * log messages to the console or to a file, depending on the build configuration.
 */

#include <cassert>
#include <iostream>

#ifdef ENABLE_DBG_ASSERT
/**
 * @brief Macro for checking conditions with operators, outputs error and aborts if condition is false
 *
 * @param name The name of the check operation
 * @param op The operator used for comparison
 * @param val1 First value to compare
 * @param val2 Second value to compare
 */
#define DBG_ASSERT_OP(name, op, val1, val2)                                                                             \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!((val1)op(val2)))                                                                                         \
        {                                                                                                              \
            std::cerr << "[ASSERT] " << __FILE__ << ":" << __LINE__ << ": Check failed: " << #val1 << " " << #op       \
                      << " " << #val2 << " (" << (val1) << " vs. " << (val2) << ")" << std::endl;                      \
            assert((val1)op(val2));                                                                                    \
        }                                                                                                              \
    } while (0)

#define DBG_ASSERT_COND(condition)                                                                                           \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(condition))                                                                                              \
        {                                                                                                              \
            std::cerr << "[ASSERT] " << __FILE__ << ":" << __LINE__ << ": Check failed: " << #condition << std::endl;  \
            assert(condition);                                                                                         \
        }                                                                                                              \
    } while (0)

#define DBG_ASSERT_EQ(val1, val2) DBG_ASSERT_OP(EQ, ==, val1, val2)
#define DBG_ASSERT_NE(val1, val2) DBG_ASSERT_OP(NE, !=, val1, val2)
#define DBG_ASSERT_LE(val1, val2) DBG_ASSERT_OP(LE, <=, val1, val2)
#define DBG_ASSERT_LT(val1, val2) DBG_ASSERT_OP(LT, <, val1, val2)
#define DBG_ASSERT_GE(val1, val2) DBG_ASSERT_OP(GE, >=, val1, val2)
#define DBG_ASSERT_GT(val1, val2) DBG_ASSERT_OP(GT, >, val1, val2)
#else
#define DBG_ASSERT_OP(name, op, val1, val2) static_cast<void>(0)
#define DBG_ASSERT_COND(condition) static_cast<void>(0)
#define DBG_ASSERT_EQ(val1, val2) static_cast<void>(0)
#define DBG_ASSERT_NE(val1, val2) static_cast<void>(0)
#define DBG_ASSERT_LE(val1, val2) static_cast<void>(0)
#define DBG_ASSERT_LT(val1, val2) static_cast<void>(0)
#define DBG_ASSERT_GE(val1, val2) static_cast<void>(0)
#define DBG_ASSERT_GT(val1, val2) static_cast<void>(0)
#endif

#define AUDIO_INFO_PRINT(fmt, ...) printf("[INF] %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define AUDIO_ERROR_PRINT(fmt, ...) printf("[ERR] %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define AUDIO_DEBUG_PRINT(fmt, ...) printf("[DEB] %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#endif