#pragma once

#include <list>
#include <numeric>
#include <algorithm>

using namespace std;

class EncoderVelocity 
{
    public: 

        EncoderVelocity(float wheelCircumference, int stepsPerRevolution, bool directionType, int filterLength, float filterConstant);

        void ProcessLeftSignal(int timestamp, bool direction, int tachometer);

        void ProcessRightSignal(int timestamp, bool direction, int tachometer);

        float GetVelocity();

    private:

        float CalculateSpeed(int ts1, int ts2, int tacho1, int tacho2, int direction);

        void PushBuffer(std::list<float> &buffer, float element);

        std::list<float> _buffer_left;
        std::list<float> _buffer_right;

        int _time_left_last = 0;
        int _time_right_last = 0;

        int _tacho_right_last = 0;
        int _tacho_left_last = 0;

        float _tickDistance;
        bool _forwardDirection;
        int _filterLength;
        float _filterConstant;

        float _last_speed = 0;

}; 