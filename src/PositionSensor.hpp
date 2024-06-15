#ifndef H_POSITION_SENSOR_
#define H_POSITION_SENSOR_

#include "LowpassFilter.hpp"

extern const uint64_t SENSOR_PERIOD_US;

const float LPF_CUTOFF_FREQ = 5;

class PositionSensor
{
private:
    LowpassFilter lpf;

public:

    PositionSensor(float low_threshold, float high_threshold);

    float low_threshold = 3000;
    float high_threshold = 3500;

    enum Flank { FLANK_NONE, FLANK_POSITIVE, FLANK_NEGATIVE };

    bool has_changed_state = false;
    Flank last_flank = FLANK_NONE;
    float last_fixed_position = -1;

    float filtered_val;
    float raw_val;
    uint8_t state;

    PositionSensor();

    void update(float  value, int32_t current_speed);

    void reset();
};

#endif