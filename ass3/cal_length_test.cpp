#include <iostream>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;
using namespace std;

int main(int argc, const char** argv){
    Vector3f a = Vector3f(1,1,1);
    cout << a.norm() << endl;

    return 0;
}