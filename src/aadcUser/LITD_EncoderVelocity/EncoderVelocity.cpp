#include "EncoderVelocity.h"

using namespace std;

float median(std::list<float> &buffer);

// if directionType is false, then direction false means forward is false
EncoderVelocity::EncoderVelocity(float wheelCircumference, int stepsPerRevolution, bool forwardDirection, int filterLength, float filterConstant) {
    _forwardDirection = forwardDirection;
    _filterLength = filterLength;
    _filterConstant = filterConstant;

    _tickDistance = static_cast<float>(wheelCircumference) / stepsPerRevolution;
}

void EncoderVelocity::ProcessLeftSignal(int timestamp, bool direction, int tachometer) {
    float speed = CalculateSpeed(_time_left_last, timestamp, _tacho_left_last, tachometer, direction);
    
    PushBuffer(_buffer_left, speed);

    _time_left_last = timestamp;
    _tacho_left_last = tachometer;
}

void EncoderVelocity::ProcessRightSignal(int timestamp, bool direction, int tachometer) {
    float speed = CalculateSpeed(_time_right_last, timestamp, _tacho_right_last, tachometer, direction);
    
    PushBuffer(_buffer_right, speed);

    _time_right_last = timestamp;
    _tacho_right_last = tachometer;
}

inline float EncoderVelocity::CalculateSpeed(int ts1, int ts2, int tacho1, int tacho2, int direction) {
    float sign = direction == _forwardDirection ? 1.0f : -1.0f;

    float dS = static_cast<float>(tacho2 - tacho1) * _tickDistance;
    float dT= static_cast<float>(ts2 - ts1) / 1e6;

    float speed = (dS / dT) * sign;
    return speed;
}

inline void EncoderVelocity::PushBuffer(std::list<float> &buffer, float element) {
	buffer.push_back(element);
    if (buffer.size() > _filterLength) {
        buffer.pop_front();
    }
}

float EncoderVelocity::GetVelocity() {
    // average filter
    // float speed_left = static_cast<float>(std::accumulate(std::begin(_buffer_left), std::end(_buffer_left), 0.0)) / _buffer_left.size();
    // float speed_right = static_cast<float>(std::accumulate(std::begin(_buffer_right), std::end(_buffer_right), 0.0)) / _buffer_right.size();

    // median filter
    float speed_left = median(_buffer_left);
    float speed_right = median(_buffer_right);

    float speed = (speed_left + speed_right) / 2;

    // pt1 filter
    speed = (_last_speed + _filterConstant * speed) / (1.0f + _filterConstant);
    _last_speed = speed;

    return speed;
}

float median(std::list<float> &buffer) {
    int size = buffer.size();

    float* arr = new float[size];
    copy(buffer.begin(), buffer.end(), arr);
    std::sort(arr, arr + size);

    if (size % 2 == 0) {
        return (arr[size / 2] + arr[(size / 2) - 1]) / 2;
    } else {
        return arr[size / 2];
    }
}