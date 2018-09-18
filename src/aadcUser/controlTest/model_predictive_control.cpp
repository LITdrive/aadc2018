#include <math.h>
#include "model_predictive_control.h"

ModelPredictiveControl::ModelPredictiveControl()
{
	ModelPredictiveControl(0.1, 1.0);
}

ModelPredictiveControl::ModelPredictiveControl(double dtime, double carLength)
{
	m_dtime = dtime;
	m_carLength = carLength;

	m_Q = 0.5, 0.5, 1, 1;
	m_R = 0.01, 0.01;

    double minAccel = -1.0;
    double maxAccel = 1.0;
    double minSteerAngle = -M_PI / 4.0;
    double maxSteerAngle =  M_PI / 4.0;

    m_maxSteerLimit = maxSteerAngle;
    m_minSteerLimit = minSteerAngle;
    m_minSpeedLimit = -100.0;
    m_maxSpeedLimit = 100.0;


	m_lowerBound = minAccel, minSteerAngle;
	m_upperBound = maxAccel, maxSteerAngle;

	m_Kp = 1.0;
	m_Kstanely = 0.5;

}

void ModelPredictiveControl::UpdateLinearModelMatrix(double v, double phi, double delta)
{
    m_A(0, 0) = 1.0;
    m_A(0, 0) = 0.0;
    m_A(0, 2) = m_dtime * cos(phi);
    m_A(0, 3) = -m_dtime * v * sin(phi);
    m_A(1, 0) = 0.0;
    m_A(1, 1) = 1.0;
    m_A(1, 2) = m_dtime * sin(phi);
    m_A(1, 3) = m_dtime * v * cos(phi);
    m_A(2, 0) = 0.0;
    m_A(2, 1) = 0.0;
    m_A(2, 2) = 1.0;
    m_A(2, 3) = 0.0;
    m_A(3, 0) = 0.0;
    m_A(3, 1) = 0.0;
    m_A(3, 2) = m_dtime * tan(delta) * m_carLength;
    m_A(3, 3) = 1.0;

    m_B(0, 0) = 0.0;
    m_B(0, 1) = 0.0;
    m_B(1, 0) = 0.0;
    m_B(1, 1) = 0.0;
    m_B(2, 0) = m_dtime;
    m_B(2, 1) = 0.0;
    m_B(3, 0) = 0.0;
    m_B(3, 1) = m_dtime * v / (m_carLength * cos(delta) * cos(delta));

    m_C(0) = m_dtime * v * sin(phi) * phi;
    m_C(1) = -m_dtime * v * cos(phi) * phi;
    m_C(2) = 0.0;
    m_C(3) = v * delta / (m_carLength * cos(delta) * cos(delta));

//    m_A = 1, 0, 1, 0,  // next_pos = pos + velocity
//        0, 1, 0, 1,  // next_pos = pos + velocity
//        0, 0, 1, 0,  // next_velocity = velocity
//        0, 0, 0, 1;  // next_velocity = velocity
//
//    // Here we say that the control variables effect only the velocity. That is,
//    // the control applies an acceleration to the vehicle.
//    m_B = 0, 0,
//        0, 0,
//        1, 0,
//        0, 1;
//
//    // Let's also say there is a small constant acceleration in one direction.
//    // This is the force of gravity in our model.
//    m_C = 0,
//        0,
//        0,
//        0.1;
}

void ModelPredictiveControl::InitState(dlib::matrix<double, 4, 1> &x0)
{
	m_state = x0;
}

dlib::matrix<double, 4, 1> ModelPredictiveControl::UpdateState(double a, double delta)
{
    if(delta >= m_maxSteerLimit)
    {
        delta = m_maxSteerLimit;
    }
    else if(delta <= m_minSteerLimit)
    {
        delta = m_minSteerLimit;
    }

    m_state(0) = m_state(0) + m_state(2) * cos(m_state(3)) * m_dtime;
    m_state(1) = m_state(1) + m_state(2) * sin(m_state(3)) * m_dtime;
    m_state(2) = m_state(2) + a * m_dtime;
    m_state(3) = m_state(3) + m_state(2) * tan(delta) * m_dtime / m_carLength;

    if(m_state(2) > m_maxSpeedLimit)
    {
    	m_state(2) = m_maxSpeedLimit;
    }
    else if(m_state(2) < m_minSpeedLimit)
    {
    	m_state(2) = m_minSpeedLimit;
    }

//	m_state = m_A * m_state + m_B * m_action + m_C;

    return m_state;
}

void ModelPredictiveControl::SetTarget(dlib::matrix<double, 4, 1> &target)
{
	m_target = target;

}

void ModelPredictiveControl::Control()
{
	UpdateLinearModelMatrix(m_target(2), m_target(3), 0.0);
//	UpdateLinearModelMatrix(m_state(2), m_state(3), m_delta);
//	dlib::mpc<4, 2, 3> controller(m_A, m_B, m_C, m_Q, m_R, m_lowerBound, m_upperBound);
//	controller.set_target(m_target);
//	m_action = controller(m_state);

	m_error = m_target - m_state;

	double target_speed = m_target(2);
	double distance_error = sqrt(m_error(0) * m_error(0) + m_error(1) * m_error(1));
	double yaw_error = atan2(m_error(1) / m_error(2));

	if()

	if(fabs(m_error(3)) -  < M_PI/2.0)
	{

		target_speed += distance_error
	}
	else
	{
		target_speed
	}

	m_action(0) = m_Kp * (m_target(2) - m_state(2)); //pid control
	m_action(1) = StanelyControl();

	m_acceleration = m_action(0);
	m_delta = m_action(1);

//	m_state = m_A * m_state + m_B * m_action + m_C;

	UpdateState(m_acceleration, m_delta);
}

double ModelPredictiveControl::StanelyControl()
{
	double fx = m_state(0) + m_carLength * cos(m_state(3));
	double fy = m_state(1) + m_carLength * cos(m_state(3));

	double cx = m_target(0);
	double cy = m_target(1);

	double dx = fx - cx;
	double dy = fy - cy;

	double d = sqrt(dx * dx + dy * dy);
	double error_front = d;
	double target_yaw = atan2(fy - cy, fx - cx);

	if(target_yaw > 0.0)
	{
		error_front = - error_front;
	}

	double theta_e = m_target(3) - m_state(3);
	double theta_d = atan2(m_Kstanely * error_front, m_state(2));

	double delta = theta_e + theta_d;

	return delta;
}
//void ModelPredictiveControl::SolveMPC()
//{
//
//    // make it so MM == trans(K)*Q*(M-target)
//    M[0] = A_m * initial_state + C;
//    for (unsigned long i = 1; i < horizon; ++i)
//    {
//        M[i] = A*M[i-1] + C;
//    }
//    for (unsigned long i = 0; i < horizon; ++i)
//    {
//        M[i] = diagm(Q)*(M[i]-target[i]);
//    }
//    for (long i = (long)horizon-2; i >= 0; --i)
//    {
//        M[i] += trans(A)*M[i+1];
//    }
//    for (unsigned long i = 0; i < horizon; ++i)
//    {
//        MM[i] = trans(B)*M[i];
//    }
//
//
//
//    unsigned long iter = 0;
//    for (; iter < max_iterations; ++iter)
//    {
//        // compute current gradient and put it into df.
//        // df == H*controls + MM;
//        M[0] = B*controls[0];
//        for (unsigned long i = 1; i < horizon; ++i)
//            M[i] = A*M[i-1] + B*controls[i];
//        for (unsigned long i = 0; i < horizon; ++i)
//            M[i] = diagm(Q)*M[i];
//        for (long i = (long)horizon-2; i >= 0; --i)
//            M[i] += trans(A)*M[i+1];
//        for (unsigned long i = 0; i < horizon; ++i)
//            df[i] = MM[i] + trans(B)*M[i] + diagm(R)*controls[i];
//
//
//
//        // Check the stopping condition, which is the magnitude of the largest element
//        // of the gradient.
//        double max_df = 0;
//        unsigned long max_t = 0;
//        long max_v = 0;
//        for (unsigned long i = 0; i < horizon; ++i)
//        {
//            for (long j = 0; j < controls[i].size(); ++j)
//            {
//                // if this variable isn't an active constraint then we care about it's
//                // derivative.
//                if (!((controls[i](j) <= lower(j) && df[i](j) > 0) ||
//                      (controls[i](j) >= upper(j) && df[i](j) < 0)))
//                {
//                    if (std::abs(df[i](j)) > max_df)
//                    {
//                        max_df = std::abs(df[i](j));
//                        max_t = i;
//                        max_v = j;
//                    }
//                }
//            }
//        }
//        if (max_df < eps)
//            break;
//
//
//
//        // We will start out by doing a little bit of coordinate descent because it
//        // allows us to optimize individual variables exactly.  Since we are warm
//        // starting each iteration with a really good solution this helps speed
//        // things up a lot.
//        const unsigned long smo_iters = 50;
//        if (iter < smo_iters)
//        {
//            if (Q_diag[max_t](max_v) == 0) continue;
//
//            // Take the optimal step but just for one variable.
//            controls[max_t](max_v) = -(df[max_t](max_v)-Q_diag[max_t](max_v)*controls[max_t](max_v))/Q_diag[max_t](max_v);
//            controls[max_t](max_v) = put_in_range(lower(max_v), upper(max_v), controls[max_t](max_v));
//
//            // If this is the last SMO iteration then don't forget to initialize v
//            // for the gradient steps.
//            if (iter+1 == smo_iters)
//            {
//                for (unsigned long i = 0; i < horizon; ++i)
//                    v[i] = controls[i];
//            }
//        }
//        else
//        {
//            // Take a projected gradient step.
//            for (unsigned long i = 0; i < horizon; ++i)
//            {
//                v_old[i] = v[i];
//                v[i] = dlib::clamp(controls[i] - 1.0/lambda * df[i], lower, upper);
//                controls[i] = dlib::clamp(v[i] + (std::sqrt(lambda)-1)/(std::sqrt(lambda)+1)*(v[i]-v_old[i]), lower, upper);
//            }
//        }
//    }
//}


dlib::matrix<double, 4, 1> ModelPredictiveControl::GetState() const
{
	return m_state;
}

dlib::matrix<double, 2, 1> ModelPredictiveControl::GetAction() const
{
	return m_action;
}



//
//CarModelState ModelPredictiveControl::InitState(const CarModelState &x0)
//{
//	m_state = x0;
//	return m_state;
//}
//

//
//void SetInputBoundaries(double minSteerAngle, double maxSteerAngle, double minAccel, double maxAccel)
//{
//	m_lowerBound = minAccel, minSteerAngle;
//	m_upperBound = maxAccel, maxSteerAngle,
//}
//
//
//void ModelPredictiveControl::UpdateCostFunction()
//{
//	m_Q = 1, 1, 0.5, 0.5;
//	m_R = 0.01, 0.01;
//}
//
//void ModelPredictiveControl::PredictMotion(double vRef)
//{
//
//}
//
//void ModelPredictiveControl::Control(double vRef, double phiRef, double deltaRef)
//{
//	UpdateLinearModelMatrix(vRef, phiRef, deltaRef);
//	UpdateCostFunction();
//    dlib::mpc<4, 2, mpc_horizon> controller(m_A, m_B, m_C, m_Q, m_R, m_lowerBound, m_upperBound);
//
//    controller.set_target(m_predictedState.X, m_predictedState.Y, m_predictedState.V, m_predictedState.Yaw);
//    dlib::matrix<double, 4, 1> currentState = state.X, state.Y, state.V, state.Yaw;
//    dlib::matrix<double, 2, 1> action = controller(currentState);
//
//
//}
//
//void ModelPredictiveControl::Solve(Eigen::vectorXd coeffs)
//{
//  // Set the number of model variables (includes both states and inputs).
//  // For example: If the state is a 4 element vector, the actuators is a 2
//  // element vector and there are 10 timesteps. The number of variables is:
//  //  size_t i;
//  size_t n_vars = N * 6 + (N - 1) * 2;
//}

