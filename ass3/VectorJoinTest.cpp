#include <eigen3/Eigen/Eigen>
#include<iostream>
using namespace Eigen;
using namespace std;

int main(int argc, const char** argv){

    MatrixXf TBN(3,3);
    Vector3f t(1,1,1);
    Vector3f b(2,2,2);
    Vector3f n(3,3,3);
    TBN << t, b, n;
    cout << TBN << endl;
    
    return 0;
}