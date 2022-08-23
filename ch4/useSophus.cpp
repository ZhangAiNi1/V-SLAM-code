#include<iostream>
#include<Eigen/Geometry>
#include"sophus/so3.hpp"
#include"sophus/se3.hpp"
#include<cmath>                         //有M_PI

using namespace std;
using namespace Eigen;

int main(int argc , char** argv)
{
          Matrix3d R = AngleAxisd(M_PI / 2 , Vector3d(0,0,1)).toRotationMatrix();
          Quaterniond q(R);

          //Sophus现在是模板库，另外SO3可以由旋转矩阵也可以由四元数构造
          Sophus::SO3d SO3_R(R);
          Sophus::SO3d SO3_q(q);

          cout << "SO(3) from matrix :\n" << SO3_R.matrix() << endl;
          cout << "SO(3) from Quaternion: \n" << SO3_q.matrix() << endl;
          
          //对数影射求李代数
          Vector3d so3 = SO3_R.log();
          cout << "so3 = " << so3.transpose() << endl;
          //李代数向量到矩阵变换：SO3d中hat函数可以取向量的反对称矩阵（小帽子）
          cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;
          //李代数矩阵到向量变换：SO3d中vee函数（vee表示v符号，也就是降纬）
          cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

          //增量扰动模型更新（对SO3群左成一个德尔塔SO3）,同时也介绍了指数影射，在Sophus::SO3d中。
          Vector3d update_so3(1e-4,0,0);
          Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
          cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;

          cout << "^*********************&********&*****************************^ " << endl;
          //接下来是对SE（3)的操作,同样也可以用旋转矩阵或四元数构造。
          Vector3d t(1,0,0);  
          Sophus::SE3d SE3_Rt (R,t);
          Sophus::SE3d SE3_qt (q,t);
          cout << "SE(3) from matrix :" << SE3_Rt.matrix() << endl;
          cout << "SE(3) from quaterniond : " << SE3_qt.matrix() << endl;

          typedef Matrix<double,6,1> Vector6d;
          Vector6d se3 = SE3_Rt.log();
          cout << "se3 = " << se3 << endl;
          //在sopuhs中，平移在前，旋转（so3）在后
          
          cout << "se3 hat = " << Sophus::SE3d::hat(se3) << endl;
          cout << "se3 hat vee  = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)) << endl;

          //增量扰动模型更新，类似上面的SO3
          Vector6d update_se3 = Vector6d::Zero();
          update_se3(0,0) = double(1e-4);
          Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt ;
          cout << "SE3 updated : " << SE3_updated.matrix() << endl;
          
          return 0;

}
