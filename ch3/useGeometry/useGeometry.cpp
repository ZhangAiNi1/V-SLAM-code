/*
          主要介绍了旋转矩阵、旋转向量、欧拉角、四元数在变成中的创造和相互转换。
          构造：
                    旋转矩阵：
                              一般是三维空间的旋转，故使用Matrix3d或者Matrix3f构建即可，
                              一般在构建的时候使用Matrix3d::Identity()将其初始化为单位矩阵。
                    旋转向量：
                              使用AngleAxisd类在构造对象的时候进行赋值，这个类见名知意，角度、轴、double，
                              故第一个参数是旋转角度（弧度）,第二个参数是旋转轴（旋转轴必须是单位向量！！）
                    四元数：
                              使用Quaterniond进行构造，可以用旋转矩阵或者旋转向量构造。
                              使用q.coeffs()可以将其转化为向量（x,y,z,w）其中w是实部，coefficient是系数的意思
                              用旋转矩阵或者旋转向量构造的四元数不需要归一化，因为本身就是单位四元数
                              使用参数构造的四元数，如Quaterniond q(0.23,0.12,0.24,0.23)构造的需要进行q.normalize()归一化。
          转换：
                    旋转向量v  to ：
                              旋转矩阵：v.matrix() 或者v.toRotationMatrix();注：可以查看matrix()源码，他实现方法是返回toRotationMatrix
                                                  旋转矩阵rotation_matrix to:
                              欧拉角：matrix.eulerAngles(2，1，0),表示z,y,x返回欧拉角
                                                   输入的三个参数是0,1,2中的数字，0代表x轴，1代表y轴，2代表z轴
                                                  返回三个参数，代表旋转的弧度，范围分别是：[0 - pi] x [-pi - pi] x [-pi - pi]
          
          变换矩阵T：
                    构造：
                              使用Isometry3d进行构造，出来的是一个4x4的矩阵
                              使用T.rotate()对R进行赋值，使用T.translate(),对t进行赋值
                              在初始化T的时候，可以直接类似T.pretranslate(Vector3d(1,3,4))进行直接构造。
                              注意在输出T的时候需要使用T.matrix()将其转化为矩阵，因为没有对<<进行重载。
          旋转矩阵，旋转向量，变换矩阵，四元数对向量的变化：
                    四者做了乘法重载，均可直接使用*号。
                              
                                                 
                              
*/

#include<iostream>
#include<cmath>
#include<Eigen/Core>
#include<Eigen/Geometry>

using namespace Eigen;
using namespace std;

int main(int argc,char** argv)
{
          //rotation_vector to rotation_matrix
          //identity是将其变成单位矩阵
          Matrix3d rotation_matrix = Matrix3d::Identity();
          //AngleAxisd根据官网介绍，第一个参数是旋转角度（弧度），第二个参数是旋转轴（旋转轴必须是单位向量！！）
          AngleAxisd rotation_vector(M_PI / 4, Vector3d(0,0,1));      //沿Z轴转45度
          cout.precision(3); //cout的输出结果保留小数点后三位。
          cout << "rotation matrix = \n" << rotation_vector.matrix() << endl;             
          rotation_matrix = rotation_vector.toRotationMatrix();

          //rotation_vector 坐标变换
          Vector3d v(1,0,0);
          Vector3d v_rotated = rotation_vector * v ;
          cout << "v after rotation (by angle axis) = \n" << v_rotated.transpose() << endl;

          //rotation_matrix 坐标变换
          v_rotated = rotation_matrix * v;
          cout << "v after rotation (by rotation matrix) " << v_rotated.transpose() << endl;

          //rotation_matrix to 欧拉角
          Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);
          cout << "yaw picth roll  :\n" << euler_angles.transpose() << endl;

          //变换矩阵T
          Isometry3d T = Isometry3d::Identity();
          T.rotate(rotation_vector);
          T.pretranslate(Vector3d(1,3,4));
          cout << "Transform matrix = \n" << T.matrix() << endl;

          //变换矩阵进行坐标变换
          Vector3d vector_transformed = T * v;
          cout << "v transformed = \n" << vector_transformed.transpose() << endl;

          //四元数
          Quaterniond q(rotation_vector);
          cout << "quaternion q = \n" << q.coeffs().transpose() << endl;
          
          //四元数对向量变化
          v_rotated = q * v;
          cout << "v after rotation = \n" << v_rotated.transpose() << endl;

          //四元数在不使用*号的重载为：
          v_rotated = (q * Quaterniond(0,1,0,0) * q.inverse()).coeffs();
          //注意：四元数在对向量操作时，先把向量变成一个纯虚四元数，然后操作，得出的是Isometry3d类型的，需要用
          //.coeffs()方法将其转化为向量。
          cout << "also equal = \n" << v_rotated.transpose() << endl;









          return 0;

}
