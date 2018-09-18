//------------------- hpp Part

#pragma once

#include <vector>
#include <dlib/control.h>

class ModelPredictiveControl
{
    public:
        ModelPredictiveControl();
        ModelPredictiveControl(double dtime, double carLength);

        void UpdateLinearModelMatrix(double v, double phi, double delta);
        void InitState(dlib::matrix<double, 4, 1> &x0);
        dlib::matrix<double, 4, 1> UpdateState(double a, double delta);

        void SetTarget(dlib::matrix<double, 4, 1> &target);
        void Control();
        double StanelyControl();

        dlib::matrix<double, 4, 1> GetState() const;
        dlib::matrix<double, 2, 1> GetAction() const;

//
//        CarModelState UpdateState(double a, double delta);
//
//        CarModelState PredictMotion(
        
    private:
		const int mpc_horizon = 30; //

        double m_dtime; //todo settter/construtor
        double m_carLength; //todo setter/constructor

        dlib::matrix<double, 4, 4> m_A;
        dlib::matrix<double, 4, 2> m_B;
        dlib::matrix<double, 4, 1> m_C;

        dlib::matrix<double, 4, 1> m_Q;
        dlib::matrix<double, 2, 1> m_R;

        dlib::matrix<double, 2, 1> m_lowerBound;
        dlib::matrix<double, 2, 1> m_upperBound;

        dlib::matrix<double, 4, 1> m_state;
        dlib::matrix<double, 4, 1> m_target;
        dlib::matrix<double, 4, 1> m_error;

        double m_delta;
        double m_acceleration;

        double m_Kp;
        double m_Kstanely;
        dlib::matrix<double, 2, 1> m_action;
//
//        CarModelState m_state;
//        CarModelState m_predictedState;
//        CarModelInitialState m_initialState;
//
        double m_minSteerLimit; // todo setter/constructor
        double m_maxSteerLimit; //todo setter/constructor
        double m_minSpeedLimit; //todo setter/constructor
        double m_maxSpeedLimit; //todo setter/constructor
        

};

