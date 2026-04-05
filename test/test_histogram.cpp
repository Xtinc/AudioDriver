#include "audio_driver.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

void test_check(bool condition, const std::string &test_name)
{
    if (!condition)
    {
        std::cout << "FAILED: " << test_name << std::endl;
        assert(false);
    }
    std::cout << "PASSED: " << test_name << std::endl;
}

double exact_quantile(std::vector<double> values, double x)
{
    if (values.empty())
    {
        throw std::runtime_error("exact_quantile requires non-empty input");
    }
    std::sort(values.begin(), values.end());
    if (values.size() == 1)
    {
        return values[0];
    }

    const double rank = x * static_cast<double>(values.size() - 1);
    const size_t low = static_cast<size_t>(rank);
    const size_t high = (low + 1 < values.size()) ? low + 1 : low;
    const double frac = rank - static_cast<double>(low);
    return values[low] * (1.0 - frac) + values[high] * frac;
}

void expect_near(double actual, double expected, double tol, const std::string &name)
{
    if (std::fabs(actual - expected) > tol)
    {
        std::cout << "FAILED: " << name << " actual=" << actual << " expected=" << expected << " tol=" << tol
                  << std::endl;
        assert(false);
    }
    std::cout << "PASSED: " << name << std::endl;
}

void test_quantile_invalid_argument()
{
    Histogram<16> hist;

    bool threw_neg = false;
    try
    {
        (void)hist.quantile(-1e-6);
    }
    catch (const std::invalid_argument &)
    {
        threw_neg = true;
    }

    bool threw_gt1 = false;
    try
    {
        (void)hist.quantile(1.000001);
    }
    catch (const std::invalid_argument &)
    {
        threw_gt1 = true;
    }

    test_check(threw_neg && threw_gt1, "quantile_invalid_argument_range");
}

void test_quantile_bootstrap_exactness()
{
    Histogram<10> hist; // BOOTSTRAP_TARGET = 32
    std::vector<double> samples;
    samples.reserve(20);

    for (size_t i = 0; i < 20; ++i)
    {
        double v = 0.2 + 0.01 * static_cast<double>(i % 7) + 0.0007 * static_cast<double>(i);
        samples.push_back(v);
        hist.add(v);
    }

    const std::array<double, 5> qs = {0.0, 0.1, 0.5, 0.9, 1.0};
    for (double q : qs)
    {
        const double expected = exact_quantile(samples, q);
        const double actual = hist.quantile(q);
        expect_near(actual, expected, 1e-12, "bootstrap_exactness_q=" + std::to_string(q));
    }
}

void test_quantile_transition_continuity()
{
    Histogram<10> hist; // BOOTSTRAP_TARGET = 32
    std::vector<double> samples;
    samples.reserve(64);

    for (size_t i = 0; i < 31; ++i)
    {
        const double v = -0.4 + 0.03 * static_cast<double>(i % 11) + 0.0005 * static_cast<double>(i);
        samples.push_back(v);
        hist.add(v);
    }

    const double before = hist.quantile(0.5);

    const double bridge = 0.1234;
    samples.push_back(bridge);
    hist.add(bridge); // Transition happens here.

    const double after = hist.quantile(0.5);
    const double exact = exact_quantile(samples, 0.5);

    // Transition to bucket mode should not produce a large jump.
    test_check(std::fabs(after - before) < 0.15, "transition_continuity_jump_bound");
    // After transition, estimate should still stay near exact sample median.
    expect_near(after, exact, 0.10, "transition_continuity_median_accuracy");
}

void test_quantile_monotonicity_and_bounds()
{
    Histogram<32> hist;
    std::mt19937 rng(123456);
    std::uniform_real_distribution<double> dist(-2.5, 3.0);

    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < 12000; ++i)
    {
        const double v = dist(rng);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
        hist.add(v);
    }

    double prev = hist.quantile(0.0);
    test_check(prev >= min_v - 1e-9 && prev <= max_v + 1e-9, "bounds_q0_in_sample_range");

    for (size_t i = 1; i <= 100; ++i)
    {
        const double q = static_cast<double>(i) / 100.0;
        const double cur = hist.quantile(q);
        test_check(cur + 1e-10 >= prev, "quantile_monotonic_step_" + std::to_string(i));
        test_check(cur >= min_v - 1e-9 && cur <= max_v + 1e-9, "bounds_q_in_sample_range_" + std::to_string(i));
        prev = cur;
    }
}

void test_quantile_uniform_accuracy()
{
    Histogram<40> hist;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (size_t i = 0; i < 60000; ++i)
    {
        hist.add(dist(rng));
    }

    expect_near(hist.quantile(0.1), 0.1, 0.08, "uniform_q10_accuracy");
    expect_near(hist.quantile(0.5), 0.5, 0.08, "uniform_q50_accuracy");
    expect_near(hist.quantile(0.9), 0.9, 0.08, "uniform_q90_accuracy");
}

void test_quantile_recent_data_shift()
{
    Histogram<24> hist;

    for (size_t i = 0; i < 5000; ++i)
    {
        hist.add(0.2);
    }
    const double before = hist.quantile(0.5);

    for (size_t i = 0; i < 5000; ++i)
    {
        hist.add(0.8);
    }
    const double after = hist.quantile(0.5);

    test_check(before < 0.35, "recent_shift_before_low");
    test_check(after > 0.65, "recent_shift_after_high");
    test_check(after > before + 0.2, "recent_shift_median_moves_right");
}

void test_quantile_constant_signal()
{
    Histogram<12> hist;
    for (size_t i = 0; i < 4000; ++i)
    {
        hist.add(0.5);
    }

    expect_near(hist.quantile(0.1), 0.5, 0.03, "constant_q10");
    expect_near(hist.quantile(0.5), 0.5, 0.03, "constant_q50");
    expect_near(hist.quantile(0.9), 0.5, 0.03, "constant_q90");
}

int main()
{
    std::cout << "=== Histogram Quantile 数学验证开始 ===" << std::endl;

    try
    {
        test_quantile_invalid_argument();
        test_quantile_bootstrap_exactness();
        test_quantile_transition_continuity();
        test_quantile_monotonicity_and_bounds();
        test_quantile_uniform_accuracy();
        test_quantile_recent_data_shift();
        test_quantile_constant_signal();

        std::cout << "=== 所有数学验证通过 ===" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "测试期间发生异常: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
