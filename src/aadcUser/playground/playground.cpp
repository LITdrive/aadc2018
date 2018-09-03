#include <iostream>

#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

using namespace Eigen;
using namespace std;


int main()
{   
    const Vector2d a(1.0, 0.0);
    const Vector2d b(1.0, 0.0);
    cout << "a: " << a << endl;
    cout << "b: " << b << endl;
    Vector2d result = a - b;
    cout << "result: " << result << endl;
    return 0;
}
