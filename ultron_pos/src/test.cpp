#include <Eigen/Core>
#include <iostream>

int main(){
    int a{};
    std::cout << "a = " << a << std::endl;
    Eigen::Matrix<double, 6, 1> test;
    // test.setZero();
    std::cout << "test = " << test.transpose() << std::endl;
    Eigen::Matrix3d mat{Eigen::Matrix3d::Identity()};
    std::cout << mat << std::endl;

    // std::vector<double> t(5,int(9));
    // //print t
    // for(auto i: t){
    //   std::cout<<i<<std::endl;
    // }
    return 0;
}