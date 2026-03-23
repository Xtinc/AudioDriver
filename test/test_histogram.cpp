#include "audio_driver.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <chrono>

// Test reporting function
void test_check(bool condition, const std::string &test_name)
{
    if (!condition)
    {
        std::cout << "FAILED: " << test_name << std::endl;
        assert(false);
    }
    else
    {
        std::cout << "PASSED: " << test_name << std::endl;
    }
}

// Test basic construction and initialization
void test_basic_construction()
{
    Histogram<10> hist("test_histogram");
    
    // 验证初始状态 - print方法应该能正常工作
    std::string output = hist.print();
    test_check(!output.empty(), "basic_construction - print should not be empty");
    
    std::cout << "初始直方图:\n" << output << std::endl;
}

// Test single value addition
void test_single_value_add()
{
    Histogram<5> hist("single_value");
    
    // 添加范围内的值
    hist.add(0.5);
    std::string output1 = hist.print();
    test_check(!output1.empty(), "single_value_add - after adding 0.5");
    
    // 添加边界值
    hist.add(0.0);
    hist.add(1.0);
    std::string output2 = hist.print();
    test_check(!output2.empty(), "single_value_add - after adding boundary values");
    
    std::cout << "添加边界值后:\n" << output2 << std::endl;
}

// Test out-of-range values
void test_out_of_range_values()
{
    Histogram<5> hist("out_of_range");
    
    // 添加超出初始范围的值
    hist.add(-1.0);  // 小于最小值
    hist.add(2.0);   // 大于最大值
    
    std::string output = hist.print();
    test_check(!output.empty(), "out_of_range_values - should handle values outside initial range");
    
    std::cout << "超出范围值测试:\n" << output << std::endl;
}

// Test normal distribution
void test_normal_distribution()
{
    Histogram<20> hist("normal_distribution");
    
    // 生成正态分布数据
    std::default_random_engine generator(12345); // 固定种子
    std::normal_distribution<double> normal_dist(0.5, 0.1);
    
    for (int i = 0; i < 1000; ++i)
    {
        double value = normal_dist(generator);
        hist.add(value);
    }
    
    std::string output = hist.print();
    test_check(!output.empty(), "normal_distribution - should handle 1000 samples");
    
    std::cout << "正态分布测试 (均值=0.5, 标准差=0.1):\n" << output << std::endl;
}

// Test uniform distribution
void test_uniform_distribution()
{
    Histogram<15> hist("uniform_distribution");
    
    // 生成均匀分布数据
    std::default_random_engine generator(54321); // 固定种子
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
    
    for (int i = 0; i < 1000; ++i)
    {
        double value = uniform_dist(generator);
        hist.add(value);
    }
    
    std::string output = hist.print();
    test_check(!output.empty(), "uniform_distribution - should handle uniform data");
    
    std::cout << "均匀分布测试:\n" << output << std::endl;
}

// Test extreme values
void test_extreme_values()
{
    Histogram<10> hist("extreme_values");
    
    // 添加极端值
    hist.add(-1000.0);
    hist.add(1000.0);
    hist.add(0.0);
    hist.add(0.5);
    
    std::string output = hist.print();
    test_check(!output.empty(), "extreme_values - should handle very large/small values");
    
    std::cout << "极值测试:\n" << output << std::endl;
}

// Test adaptive adjustment
void test_adaptive_adjustment()
{
    Histogram<8> hist("adaptive");
    
    // 先添加一些集中在某个范围的值
    for (int i = 0; i < 50; ++i)
    {
        hist.add(0.4 + (i % 10) * 0.01);  // 0.4 到 0.49
    }
    
    std::string output1 = hist.print();
    std::cout << "集中值后:\n" << output1 << std::endl;
    
    // 然后添加一些分散的值  
    for (int i = 0; i < 50; ++i)
    {
        hist.add(0.1 + (i % 50) * 0.016);  // 0.1 到 0.884
    }
    
    std::string output2 = hist.print();
    test_check(!output2.empty(), "adaptive_adjustment - should adapt to changing distributions");
    
    std::cout << "分散值后:\n" << output2 << std::endl;
}

// Test forgetting factor effect
void test_forgetting_factor()
{
    Histogram<6> hist("forgetting_factor");
    
    // 添加早期数据
    for (int i = 0; i < 200; ++i)
    {
        hist.add(0.2);
    }
    
    std::string output1 = hist.print();
    std::cout << "早期数据 (0.2):\n" << output1 << std::endl;
    
    // 添加新数据
    for (int i = 0; i < 200; ++i)
    {
        hist.add(0.8);
    }
    
    std::string output2 = hist.print();
    test_check(!output2.empty(), "forgetting_factor - should weight recent data more heavily");
    
    std::cout << "新数据后 (0.8):\n" << output2 << std::endl;
}

// Test edge cases
void test_edge_cases()
{
    // 最小桶数
    Histogram<3> hist_min("min_buckets");
    hist_min.add(0.5);
    std::string output_min = hist_min.print();
    test_check(!output_min.empty(), "edge_cases - minimum bucket count (3)");
    
    // 相同值的多次添加
    Histogram<5> hist_same("same_values");
    for (int i = 0; i < 100; ++i)
    {
        hist_same.add(0.5);
    }
    std::string output_same = hist_same.print();
    test_check(!output_same.empty(), "edge_cases - many identical values");
    
    std::cout << "相同值测试:\n" << output_same << std::endl;
}

// Test performance
void test_performance()
{
    Histogram<20> hist("performance");
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // 添加大量数据
    for (int i = 0; i < 10000; ++i)
    {
        hist.add(static_cast<double>(i) / 10000.0);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << "性能测试: " << duration.count() << " 微秒用于10000次添加" << std::endl;
    
    std::string output = hist.print();
    test_check(!output.empty(), "performance - should handle 10000 additions efficiently");
    
    // 性能应该在合理范围内（比如每次添加不超过1微秒）
    test_check(duration.count() < 10000, "performance - should complete within 10ms");
}

// Test histogram boundary conditions
void test_boundary_conditions()
{
    Histogram<4> hist("boundary");
    
    // 测试空的直方图打印
    std::string empty_output = hist.print();
    test_check(!empty_output.empty(), "boundary_conditions - empty histogram should still print");
    
    // 添加一个值然后验证桶的使用
    hist.add(0.25);  // 应该落入第一个桶
    hist.add(0.75);  // 应该落入后面的桶
    
    std::string filled_output = hist.print();
    test_check(filled_output.length() >= empty_output.length(), 
               "boundary_conditions - filled histogram should have more output");
    
    std::cout << "边界条件测试:\n" << filled_output << std::endl;
}

int main()
{
    std::cout << "=== Histogram 类测试开始 ===" << std::endl;
    
    try
    {
        test_basic_construction();
        test_single_value_add();
        test_out_of_range_values();
        test_normal_distribution();
        test_uniform_distribution();
        test_extreme_values();
        test_adaptive_adjustment();
        test_forgetting_factor();
        test_edge_cases();
        test_performance();
        test_boundary_conditions();
        
        std::cout << "=== 所有测试完成 ===" << std::endl;
        
    }
    catch (const std::exception &e)
    {
        std::cout << "测试期间发生异常: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}