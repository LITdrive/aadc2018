#include <dlib/gui_widgets.h>
#include <dlib/control.h>
#include <dlib/image_transforms.h>
#include <math.h>
#include <vector>
#include "model_predictive_control.h"

using namespace std;
using namespace dlib;

//  ----------------------------------------------------------------------------

void draw_car(matrix<rgb_pixel>& world, double x, double y, double phi, double delta, double width, double lenControlPointToRear, double lenControlPointToFront, double wheelWidth, double steerLen, double wheelLen)
{
    const dpoint controlPoint = point(x, y);
	draw_solid_circle(world, controlPoint, 1, 0);  //draw control point

    std::vector<dlib::point> outline;
	outline.push_back(dlib::rotate_point(controlPoint, controlPoint + point(-lenControlPointToRear,width/2), -phi));
	outline.push_back(dlib::rotate_point(controlPoint, controlPoint + point(lenControlPointToFront, width/2), -phi));
	outline.push_back(dlib::rotate_point(controlPoint, controlPoint + point(lenControlPointToFront, -width/2), -phi));
	outline.push_back(dlib::rotate_point(controlPoint, controlPoint + point(-lenControlPointToRear, -width/2), -phi));

	for(int i=0; i < outline.size()-1; i++)
	{
		draw_line (world, outline[i], outline[i+1], rgb_pixel(100, 100, 100));
	}
	draw_line (world, outline[outline.size()-1], outline[0], rgb_pixel(100, 100, 100));

	//plot wheel right
	draw_line(world, dlib::rotate_point(controlPoint, controlPoint + point(-wheelLen/2.0, wheelWidth/2), -phi),
			         dlib::rotate_point(controlPoint, controlPoint + point(wheelLen/2.0, wheelWidth/2), -phi), rgb_pixel(100, 100, 100));

	//plot wheel left
	draw_line(world, dlib::rotate_point(controlPoint, controlPoint + point(-wheelLen/2.0, -wheelWidth/2), -phi),
			         dlib::rotate_point(controlPoint, controlPoint + point(wheelLen/2.0, -wheelWidth/2), -phi), rgb_pixel(100, 100, 100));


	//plot steer wheel right
	dpoint steerPointLeft = controlPoint + point(steerLen, wheelWidth/2);
	draw_line(world, dlib::rotate_point(controlPoint, rotate_point(steerPointLeft, steerPointLeft + point(-wheelLen/2.0, 0.0), -delta), -phi),
			         dlib::rotate_point(controlPoint, rotate_point(steerPointLeft, steerPointLeft + point(wheelLen/2.0, 0.0), -delta), -phi), rgb_pixel(100, 100, 100));
//	draw_line(world, dlib::rotate_point(controlPoint, steerPointLeft + point(-wheelLen/2.0, 0.0), -phi),
//			         dlib::rotate_point(controlPoint, steerPointLeft + point(wheelLen/2.0, 0.0), -phi), rgb_pixel(100, 100, 100));

	//plot steer wheel left
	const dpoint steerPointRight = controlPoint + point(steerLen, -wheelWidth/2);
	draw_line(world, dlib::rotate_point(controlPoint, rotate_point(steerPointRight, steerPointRight + point(-wheelLen/2.0, 0.0), -delta), -phi),
			         dlib::rotate_point(controlPoint, rotate_point(steerPointRight, steerPointRight + point(wheelLen/2.0, 0.0), -delta), -phi), rgb_pixel(100, 100, 100));



}

int main()
{
	ModelPredictiveControl mpc(0.1, 25.0);
	dlib::matrix<double, 4, 1> x0;
	x0 = 200.0, 200.0, 0.0, 0.0; //init state
	mpc.InitState(x0);
	cout << "Init State: " << trans(mpc.GetState());
    // Let's tell the controller to send our vehicle to a random location.  It
    // will try to find the controls that makes the vehicle just hover at this
    // target position.
    dlib::rand rnd;
    matrix<double, 4, 1> target;
    target = 250.0, 200.0, 0.1, 0.0; //rnd.get_random_double()*400, rnd.get_random_double()*400, 0.0, rnd.get_random_double()*M_PI;

	mpc.SetTarget(target);

    // Now let's start simulating our vehicle.  Our vehicle moves around inside
    // a 400x400 unit sized world.
    matrix<rgb_pixel> world(400,400);
    image_window win;
    matrix<double,4,1> current_state;
    matrix<double,2,1> action;

    int iter = 0;
    while(!win.is_closed())
    {
    	mpc.Control();
    	action = mpc.GetAction();
    	current_state = mpc.GetState();
        cout << "best control: " << trans(action);
        cout << "current_state: " << trans(current_state);

        // Now draw our vehicle on the world.  We will draw the vehicle as a
        // black circle and its target position as a green circle.
        assign_all_pixels(world, rgb_pixel(255,255,255));
        const dpoint pos = point(current_state(0),current_state(1));

        draw_car(world, //world
        		 current_state(0), //x coordinate
				 400-current_state(1), //y coordinate
				 current_state(3), //yaw angle
				 action(1), //delta
				 30.0, //outline width
				 10.0, //lenControlPointToRear
				 30.0, //lenControlPointToFront
				 20.0, //wheelWidth
				 25.0, //steerLen
				 5.0); //wheelLen

        const dpoint goal = point(target(0),400-target(1));
        draw_solid_circle(world, goal, 9, rgb_pixel(100,255,100));
//        draw_solid_circle(world, pos, 7, 0);


        // We will also draw the control as a line showing which direction the
        // vehicle's thruster is firing.
        draw_line(world, pos, pos-50*action, rgb_pixel(255,0,0));
        win.set_image(world);

        // Take a step in the simulation
        dlib::sleep(100);

        // Every 100 iterations change the target to some other random location.
        ++iter;
        if (iter > 5)
        {
            iter = 0;
            target = target(0)+10.0, target(1)+0.0, target(2), target(3); // rnd.get_random_double()*400,rnd.get_random_double()*400,0,rnd.get_random_double()*M_PI;
            mpc.SetTarget(target);
            cout << "New Target: " << trans(target) << endl;
        }
    }
}



//  ----------------------------------------------------------------------------
