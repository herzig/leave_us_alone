#include "LowpassFilter.hpp"

LowpassFilter::LowpassFilter(float sample_period, float cutoff_frequency) 
{
    this->sample_period = sample_period;
    this->cutoff_frequency = cutoff_frequency;

    float freq_rad = cutoff_frequency * 2 * 3.1415;
    a11 = (1.0f+freq_rad*sample_period)*exp(-freq_rad*sample_period);
    a12 = sample_period*exp(-freq_rad*sample_period);
    a21 = -freq_rad*freq_rad*sample_period*exp(-freq_rad*sample_period);
    a22 = (1.0f-freq_rad*sample_period)*exp(-freq_rad*sample_period);
    b1 = (1.0f-(1.0f+freq_rad*sample_period)*exp(-freq_rad*sample_period))/freq_rad/freq_rad;
    b2 = sample_period*exp(-freq_rad*sample_period);
    
    x1 = 0.0f;
    x2 = 0.0f;
}

void LowpassFilter::reset() 
{
    x1 = 0.0f;
    x2 = 0.0f;
}

void LowpassFilter::reset(float value) 
{
    float freq_rad = cutoff_frequency * 2 * 3.1415;
    x1 = value / freq_rad / freq_rad;
    x2 = (x1-a11*x1-b1*value)/a12;
}

void LowpassFilter::set_sample_period(float period) 
{
    sample_period = period;
    float freq_rad = cutoff_frequency * 2 * 3.1415;
    
    a11 = (1.0f+freq_rad*sample_period)*exp(-freq_rad*sample_period);
    a12 = sample_period*exp(-freq_rad*sample_period);
    a21 = -freq_rad*freq_rad*sample_period*exp(-freq_rad*sample_period);
    a22 = (1.0f-freq_rad*sample_period)*exp(-freq_rad*sample_period);
    b1 = (1.0f-(1.0f+freq_rad*sample_period)*exp(-freq_rad*sample_period))/freq_rad/freq_rad;
    b2 = sample_period*exp(-freq_rad*sample_period);
}

void LowpassFilter::set_cutoff_frequency(float frequency) 
{
    cutoff_frequency = frequency;
    float freq_rad = frequency * 2 * 3.1415;
    
    a11 = (1.0f+freq_rad*sample_period) * exp(-freq_rad*sample_period);
    a12 = sample_period * exp(-freq_rad*sample_period);
    a21 = -freq_rad*freq_rad * sample_period * exp(-freq_rad*sample_period);
    a22 = (1.0f-freq_rad*sample_period) * exp(-freq_rad*sample_period);
    b1 = (1.0f-(1.0f+freq_rad*sample_period) * exp(-freq_rad*sample_period))/freq_rad/freq_rad;
    b2 = sample_period * exp(-freq_rad*sample_period);
}

float LowpassFilter::get_cutoff_frequency() 
{
    return cutoff_frequency;
}

float LowpassFilter::filter(float value) 
{
    float x1old = x1;
    float x2old = x2;
    
    x1 = a11*x1old+a12*x2old+b1*value;
    x2 = a21*x1old+a22*x2old+b2*value;
    
    float freq_rad = cutoff_frequency * 2 * 3.1415;
    return freq_rad*freq_rad*x1;
}