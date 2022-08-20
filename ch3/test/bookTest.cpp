/*
          初始化矩阵方法：
                    静态矩阵：
                              1.可以用重载后的<< 进行直接赋值，例如：
                                        Matrix<double,3,3> matrix;
                                        matrix << 1,2,3,4,5,6,7,8,9;
                              2.可以用Random()，在-1到1之间随机赋值，例如：
                                        Matrix<double,3,3> matrix = Matrix<double,3,3>::Random();
                              3.可以用Identity(),将第i行的第i个元素赋值为1，其余元素为0，例如：
                                        Matrix<double,4,4> matrix = Matrix<double,4,4>::Identity();
                              4.可以用Zero(),将矩阵元素全赋值为0
                                       Matrix<double,4,4> matrix = Matrix<double,4,4>::Zero();
                    动态矩阵：
                              1.可以通过宏定义行数和列数来模拟动态矩阵,例如：
                                        #define row 20
                                        #define col 30
                                        Matrix<double,row,col> matrix;
                              2.可以用Random()，在-1到1之间随机赋值,同时指定了动态矩阵的行数和列数，例如：
                                        MatrixXd matrix = MatrixXd::Random(5,6);
                              3.可以用Identity(),将第i行的第i个元素赋值为1，其余元素为0,同时指定了动态矩阵的行数和列数，例如：
                                        MatrixXd matrix = MatrixXd::Identity(5,6);
                              4.通过Zero(row,col)来将矩阵赋值成row,col的0矩阵，同时指定了动态矩阵的行数和列数，例如：
                                        MatrixXd matrix = MatrixXd::Zero(5,6);
                              注意：动态矩阵MatrixXd的初始化也可以像第一个一样利用宏定义行和列。
          从已有矩阵中取出其中元素的方法：
                    可使用block(Index startRow, Index startCol, Index blockRows, Index blockCols)方法，
                    startRow是起始行数，startCol是起始列数，blockRows是一共需要取几行元素，blockCols是一共需要取几行列数
                    例如：
                              MatrixXd matrix = MatrixXd::Random(30,30);
                              Matrix3d I = matrix.block(0,0,3,3);
*/

#include<Eigen/Core>
#include<iostream>

using namespace std;
using namespace Eigen;

#define SIZE 50

int main(int argc,char** argv)
{
          MatrixXd matrix = MatrixXd::Random(SIZE,SIZE);
          Matrix3d I = matrix.block(0,0,3,3);
          I = Matrix3d::Identity();

          cout << I << endl;

          return 0;

}