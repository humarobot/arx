#include "HermiteSpline.hpp"
#include "gtest/gtest.h"

class HermiteSplineTest : public ::testing::Test {
protected:
  HermiteSplineTest(){
    std::vector<KnotPoint> knots(2);
    knots[0].position << 0, 0, 0;
    knots[0].velocity << 0, 1, 0;
    knots[1].position << 1, 0, 0;
    knots[1].velocity << 0, -1, 0;
    hermite_spline_.SetKnotsTMax(knots,1.0);
  }

  HermiteSpline hermite_spline_{};

};

TEST_F(HermiteSplineTest, twoPointsPositionTest){
  //print hermite spline.getPosition(0.0)
  std::cout<<"hermite spline.getPosition(0.0) = "<<hermite_spline_.getPosition(0.0).transpose()<<std::endl;
  std::cout<<"hermite spline.getPosition(0.5) = "<<hermite_spline_.getPosition(0.5).transpose()<<std::endl;
  ASSERT_TRUE(hermite_spline_.getPosition(0.0).isApprox(Vector3d(0,0,0)));
  // ASSERT_TRUE(hermite_spline_.getPosition(0.5).isApprox(Vector3d(0.5,0,0)));
  ASSERT_TRUE(hermite_spline_.getPosition(1.0).isApprox(Vector3d(1,0,0)));
  
}

TEST_F(HermiteSplineTest, twoPointsVelTest){
  
  // std::cout<<"hermite spline.getVelocity(0.0) = "<<hermite_spline_.getVelocity(0.0).transpose()<<std::endl;
  std::cout<<"hermite spline.getVelocity(0.5) = "<<hermite_spline_.getVelocity(0.5).transpose()<<std::endl;
  
  ASSERT_TRUE(hermite_spline_.getVelocity(0.0).isApprox(Vector3d(0,1,0)));
  // ASSERT_TRUE(hermite_spline_.getVelocity(0.5).isApprox(Vector3d(0,0,0)));
  ASSERT_TRUE(hermite_spline_.getVelocity(1.0).isApprox(Vector3d(0,-1,0)));
}