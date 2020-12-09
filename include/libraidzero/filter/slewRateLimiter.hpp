#pragma once

#include <cstdint>

class SlewRateLimiter {
public:
    /**
     * Constructs a SlewRateLimiter.
     * 
     * @param irateLimit rate limit in units/s
     * @param iinitialValue starting value in units
     */
    SlewRateLimiter(double irateLimit, double iinitialValue = 0.0);
    double calculate(double input);
    void reset(double value);
private:
    double rateLimit;
    std::uint32_t previousTime;
    double previousValue;
};