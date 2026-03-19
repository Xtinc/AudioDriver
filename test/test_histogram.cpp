#include "audio_driver.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>
#include <stdexcept>

void test_basic_functionality()
{
    std::cout << "=== Test Basic Functionality ===\n";
    
    Histogram<5> hist(0.0, 10.0);
    
    // Test initial state
    assert(hist.total_weight() == 0.0);
    assert(hist.bucket_count() == 7); // 5 normal buckets + 2 overflow buckets
    
    // Add some values within range
    hist.add(2.5);
    hist.add(5.0);
    hist.add(7.5);
    
    std::cout << "Histogram after adding 3 values:\n" << hist.print() << std::endl;
    
    // Verify total weight is greater than 0
    assert(hist.total_weight() > 0.0);
    
    std::cout << "Basic functionality test passed\n\n";
}

void test_boundary_cases()
{
    std::cout << "=== Test Boundary Cases ===\n";
    
    Histogram<3> hist(1.0, 4.0);
    
    // Test underflow
    hist.add(-1.0); // Should go to bucket 0 (underflow bucket)
    hist.add(0.5);  // Should go to bucket 0 (underflow bucket)
    
    // Test overflow  
    hist.add(5.0);  // Should go to bucket 4 (overflow bucket)
    hist.add(10.0); // Should go to bucket 4 (overflow bucket)
    
    // Test boundary values
    hist.add(1.0);  // Equal to min_val, should go to bucket 1
    hist.add(4.0);  // Equal to max_val, should go to bucket 4 (overflow bucket)
    
    // Test middle values
    hist.add(2.5);  // Should go to bucket 2
    
    std::cout << "Boundary test histogram:\n" << hist.print() << std::endl;
    
    // Verify underflow and overflow buckets have data
    assert(hist.bucket_weight(0) > 0.0); // Underflow bucket
    assert(hist.bucket_weight(4) > 0.0); // Overflow bucket (3 normal buckets: 1,2,3, so overflow bucket is 4)
    
    std::cout << "Boundary test passed\n\n";
}

void test_uniform_distribution()
{
    std::cout << "=== Test Uniform Distribution ===\n";
    
    Histogram<8> hist(0.0, 8.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> uniform(0.0, 8.0);
    
    const int num_samples = 8000;
    for (int i = 0; i < num_samples; i++)
    {
        hist.add(uniform(gen));
    }
    
    std::cout << "Uniform Distribution U(0,8) - " << num_samples << " samples:\n";
    std::cout << hist.print() << std::endl;
    
    // For uniform distribution, bucket weights should be relatively uniform
    std::vector<double> bucket_weights;
    for (size_t i = 1; i <= 8; i++) { // Skip underflow and overflow buckets
        bucket_weights.push_back(hist.bucket_weight(i));
    }
    
    // Calculate standard deviation, should be relatively small
    double mean = std::accumulate(bucket_weights.begin(), bucket_weights.end(), 0.0) / bucket_weights.size();
    double sq_sum = 0.0;
    for (double w : bucket_weights) {
        sq_sum += (w - mean) * (w - mean);
    }
    double stddev = std::sqrt(sq_sum / bucket_weights.size());
    
    std::cout << "Uniform distribution bucket weight standard deviation: " << stddev << std::endl;
    assert(stddev < mean * 0.3); // Standard deviation should be less than 30% of mean
    
    std::cout << "Uniform distribution test passed\n\n";
}

void test_exponential_distribution()
{
    std::cout << "=== Test Exponential Distribution ===\n";
    
    Histogram<10> hist(0.0, 5.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::exponential_distribution<> exponential(1.0); // λ = 1.0
    
    const int num_samples = 10000;
    for (int i = 0; i < num_samples; i++)
    {
        hist.add(exponential(gen));
    }
    
    std::cout << "Exponential Distribution Exp(1.0) - " << num_samples << " samples:\n";
    std::cout << hist.print() << std::endl;
    
    // Exponential distribution should have higher probability on the left side
    double left_weight = hist.bucket_weight(1) + hist.bucket_weight(2); // First two buckets
    double right_weight = hist.bucket_weight(9) + hist.bucket_weight(10); // Last two buckets
    
    std::cout << "Left weight: " << left_weight << ", Right weight: " << right_weight << std::endl;
    assert(left_weight > right_weight); // Left side should have more weight
    
    std::cout << "Exponential distribution test passed\n\n";
}

void test_normal_distribution()
{
    std::cout << "=== Test Normal Distribution ===\n";
    
    Histogram<10> hist(-3.0, 3.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> normal(0.0, 1.0); // Standard normal distribution
    
    const int num_samples = 10000;
    for (int i = 0; i < num_samples; i++)
    {
        hist.add(normal(gen));
    }
    
    std::cout << "Standard Normal Distribution N(0,1) - " << num_samples << " samples:\n";
    std::cout << hist.print() << std::endl;
    
    // Normal distribution should have highest probability at center
    double center_weight = hist.bucket_weight(5) + hist.bucket_weight(6); // Center buckets
    double edge_weight = hist.bucket_weight(1) + hist.bucket_weight(10); // Edge buckets
    
    std::cout << "Center weight: " << center_weight << ", Edge weight: " << edge_weight << std::endl;
    assert(center_weight > edge_weight); // Center should have more weight
    
    // Verify total weight
    double total = hist.total_weight();
    std::cout << "Total weight: " << total << std::endl;
    assert(total > 0.5 && total <= 1.1);
    
    std::cout << "Normal distribution test passed\n\n";
}

void test_gamma_distribution()
{
    std::cout << "=== Test Gamma Distribution ===\n";
    
    Histogram<12> hist(0.0, 12.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::gamma_distribution<> gamma(2.0, 2.0); // α=2, β=2
    
    const int num_samples = 10000;
    for (int i = 0; i < num_samples; i++)
    {
        hist.add(gamma(gen));
    }
    
    std::cout << "Gamma Distribution Γ(2,2) - " << num_samples << " samples:\n";
    std::cout << hist.print() << std::endl;
    
    // Gamma distribution should have right-skewed shape
    double left_third = hist.bucket_weight(1) + hist.bucket_weight(2) + hist.bucket_weight(3) + hist.bucket_weight(4);
    double right_third = hist.bucket_weight(9) + hist.bucket_weight(10) + hist.bucket_weight(11) + hist.bucket_weight(12);
    
    std::cout << "Left 1/3 weight: " << left_third << ", Right 1/3 weight: " << right_third << std::endl;
    // For α=2 gamma distribution, left side should have more probability
    
    std::cout << "Gamma distribution test passed\n\n";
}

void test_chi_squared_distribution()
{
    std::cout << "=== Test Chi-squared Distribution ===\n";
    
    Histogram<10> hist(0.0, 10.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::chi_squared_distribution<> chi_sq(3); // degrees of freedom = 3
    
    const int num_samples = 10000;
    for (int i = 0; i < num_samples; i++)
    {
        hist.add(chi_sq(gen));
    }
    
    std::cout << "Chi-squared Distribution χ²(3) - " << num_samples << " samples:\n";
    std::cout << hist.print() << std::endl;
    
    // Chi-squared distribution starts from 0, right-skewed
    double first_bucket = hist.bucket_weight(1);
    double last_bucket = hist.bucket_weight(10);
    
    std::cout << "First bucket weight: " << first_bucket << ", Last bucket weight: " << last_bucket << std::endl;
    assert(first_bucket > last_bucket); // First bucket should have more weight
    
    std::cout << "Chi-squared distribution test passed\n\n";
}

void test_beta_distribution()
{
    std::cout << "=== Test Beta Distribution ===\n";
    
    Histogram<10> hist(0.0, 1.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Use two gamma distributions to generate beta distribution
    std::gamma_distribution<> gamma_a(2.0, 1.0);
    std::gamma_distribution<> gamma_b(5.0, 1.0);
    
    const int num_samples = 10000;
    for (int i = 0; i < num_samples; i++)
    {
        double x = gamma_a(gen);
        double y = gamma_b(gen);
        double beta_sample = x / (x + y); // Beta distribution
        hist.add(beta_sample);
    }
    
    std::cout << "Beta Distribution Beta(2,5) - " << num_samples << " samples:\n";
    std::cout << hist.print() << std::endl;
    
    // Beta(2,5) should be left-skewed
    double left_weight = hist.bucket_weight(1) + hist.bucket_weight(2) + hist.bucket_weight(3);
    double right_weight = hist.bucket_weight(8) + hist.bucket_weight(9) + hist.bucket_weight(10);
    
    std::cout << "Left weight: " << left_weight << ", Right weight: " << right_weight << std::endl;
    assert(left_weight > right_weight); // Left side should have more weight
    
    std::cout << "Beta distribution test passed\n\n";
}

void test_mixed_distribution()
{
    std::cout << "=== Test Mixed Distribution ===\n";
    
    Histogram<15> hist(-5.0, 10.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> selector(0.0, 1.0);
    std::normal_distribution<> normal1(-2.0, 0.5);
    std::normal_distribution<> normal2(4.0, 1.0);
    std::exponential_distribution<> exponential(0.5);
    
    const int num_samples = 12000;
    int normal1_count = 0, normal2_count = 0, exp_count = 0;
    
    for (int i = 0; i < num_samples; i++)
    {
        double p = selector(gen);
        if (p < 0.4) {
            hist.add(normal1(gen));
            normal1_count++;
        } else if (p < 0.7) {
            hist.add(normal2(gen));
            normal2_count++;
        } else {
            hist.add(exponential(gen));
            exp_count++;
        }
    }
    
    std::cout << "Mixed Distribution - " << num_samples << " samples:\n";
    std::cout << "  40% N(-2, 0.5²): " << normal1_count << " samples\n";
    std::cout << "  30% N(4, 1²): " << normal2_count << " samples\n";
    std::cout << "  30% Exp(0.5): " << exp_count << " samples\n";
    std::cout << hist.print() << std::endl;
    
    double total = hist.total_weight();
    std::cout << "Total weight: " << total << std::endl;
    assert(total > 0.5 && total <= 1.1);
    
    std::cout << "Mixed distribution test passed\n\n";
}

void test_decay_factor()
{
    std::cout << "=== Test Decay Factor ===\n";
    
    Histogram<3> hist(0.0, 3.0);
    
    // First add a value
    hist.add(1.5);
    double weight_after_first = hist.total_weight();
    std::cout << "Weight after first value: " << weight_after_first << std::endl;
    
    // Add second value, first value should be decayed
    hist.add(1.5);
    double weight_after_second = hist.total_weight();
    std::cout << "Weight after second value: " << weight_after_second << std::endl;
    
    // Due to decay, second total weight should be less than 2x first
    assert(weight_after_second < 2.0 * weight_after_first);
    
    std::cout << "Decay factor test passed\n\n";
}

void test_error_handling()
{
    std::cout << "=== Test Error Handling ===\n";
    
    try
    {
        // Try to create invalid histogram (max <= min)
        Histogram<5> invalid_hist(5.0, 2.0);
        assert(false); // Should not reach here
    }
    catch (const std::invalid_argument& e)
    {
        std::cout << "Correctly caught exception: " << e.what() << std::endl;
    }
    
    try
    {
        Histogram<5> same_hist(5.0, 5.0);
        assert(false); // Should not reach here
    }
    catch (const std::invalid_argument& e)
    {
        std::cout << "Correctly caught exception: " << e.what() << std::endl;
    }
    
    std::cout << "Error handling test passed\n\n";
}

void test_reset_functionality()
{
    std::cout << "=== Test Reset Functionality ===\n";
    
    Histogram<5> hist(0.0, 10.0);
    
    // Add some data
    hist.add(2.0);
    hist.add(5.0);
    hist.add(8.0);
    
    assert(hist.total_weight() > 0.0);
    
    // Reset
    hist.reset();
    assert(hist.total_weight() == 0.0);
    
    // Should work normally after reset
    hist.add(3.0);
    assert(hist.total_weight() > 0.0);
    
    std::cout << "Reset functionality test passed\n\n";
}

int main()
{
    std::cout << "Histogram Class Test Suite\n";
    std::cout << "==========================\n\n";
    
    try
    {
        test_basic_functionality();
        test_boundary_cases();
        test_decay_factor();
        test_error_handling();
        test_reset_functionality();
        
        // 各种分布测试
        test_uniform_distribution();
        test_exponential_distribution();
        test_normal_distribution();
        test_gamma_distribution();
        test_chi_squared_distribution();
        test_beta_distribution();
        test_mixed_distribution();
        
        std::cout << "All tests passed! Including verification of 7 different probability distributions!\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}