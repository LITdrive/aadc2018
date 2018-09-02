#include <iostream>
#include <memory>

using namespace std;


int main()
{   
    double test = 1.0;
    shared_ptr<double> p_test = make_shared<double> test;
    cout << "Test: " << test << " Ptr: "<< *p_test << endl;
    return 0;
}
