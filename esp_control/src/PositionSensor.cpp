#include "PositionSensor.hpp"

PositionSensor::PositionSensor(float low_threshold, float high_threshold)
    : lpf(SENSOR_PERIOD_US / 1e6f, LPF_CUTOFF_FREQ)
{
    this->low_threshold = low_threshold;
    this->high_threshold = high_threshold;
}

PositionSensor::PositionSensor(/* args */)
    : lpf(SENSOR_PERIOD_US / 1e6f, LPF_CUTOFF_FREQ)
{
}

void PositionSensor::update(float ain_value, int32_t current_speed)
{
    raw_val = ain_value;
    uint8_t prev_state = state;

    filtered_val = lpf.filter(ain_value);

    if (filtered_val < low_threshold)
        state = 0; // white
    if (filtered_val > high_threshold)
        state = 1; // black

    has_changed_state = prev_state != state;
    if (has_changed_state)
    {
        if (state == 0) 
        {
            last_flank = FLANK_NEGATIVE;
            if (current_speed < 0)
                last_fixed_position = 0;
            else
                last_fixed_position = 180;
        }
        else
        {
            last_flank = FLANK_POSITIVE;
            if (current_speed < 0)
                last_fixed_position = 180;
            else
                last_fixed_position = 0;
        }

        
    }
}

void PositionSensor::reset()
{
    has_changed_state = false;
}
