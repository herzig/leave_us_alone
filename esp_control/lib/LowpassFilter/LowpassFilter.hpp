#ifndef H_LOWPASS_FILTER_
#define H_LOWPASS_FILTER_

#include <cmath>

class LowpassFilter {
    
    public:
        LowpassFilter(float sample_period_s, float cutoff_frequency_hz);
        void reset();
        void reset(float value);
        void set_sample_period(float period);
        void set_cutoff_frequency(float frequency);
        float get_cutoff_frequency();
        float filter(float value);
        
    private:
        
        float sample_period;
        float cutoff_frequency;
        float a11, a12, a21, a22, b1, b2;
        float x1, x2;
};


#endif