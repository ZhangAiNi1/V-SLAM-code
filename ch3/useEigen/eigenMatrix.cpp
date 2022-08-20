#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 50

int main(int argc, char  *argv[])
{
          Matrix<float,2,3> matrix_23;
          Vector3d v_3d;
          Matrix<float,3,1> vd_3d;
          Matrix3d matrix_33 = Matrix3d::Zero();
          Matrix<double,Dynamic,Dynamic> matrix_dynamic;
          MatrixXd matrix_x;
          
          matrix_23 << 1,2,3,4,5,6;
          cout << "matrix 2x3 form 1 to 6 : \n" << matrix_23 << endl;
          cout << "print matrix 2x3 :" << endl;
          for(int i  = 0; i < 2 ;i++)
          {
                    for(int j = 0 ; j < 3 ; j++)
                    {
                              cout << matrix_23(i,j) << "\t" ;
                    }
                    cout << endl;
          }

          v_3d << 3,2,1;
          vd_3d << 4,5,6;
          Matrix<double,2,1> result = matrix_23.cast<double>() * v_3d;
          cout << "matrix_23*v_3d = \n" << result << endl;
          Matrix<float,2,1>result2 = matrix_23 * vd_3d;
          cout << "matrix_23 *vd_3d = \n" << endl;

          matrix_33 = Matrix3d::Random();
          cout << "random matrix :\n" << matrix_33 << endl;
          cout << "transpose :\n" << matrix_33.transpose() << endl;
          cout << "sum :\n" << matrix_33.sum() << endl;
          cout << "trace:\n" << matrix_33.trace() << endl;
          cout << "10*matrix_33 = \n" << 10 * matrix_33 << endl;
          cout << "inverse:\n" << matrix_33.inverse() << endl;
          cout << "det =\n" << matrix_33.determinant() << endl;

          //A的转置乘A，结果是对称矩阵，下面方法是在构造SelfAdjointEigenSolver对象时就进行了特征值和特征向量的计算，之后
          //使用eigenvlaues或者eigenvectors返回特征值和特征向量即可。
          SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
          cout << "Eigenvalues:  " << eigen_solver.eigenvalues() << endl;
          cout << "Eigenvectors: " << eigen_solver.eigenvectors() << endl;

          //求解 matrix_NN * x = v_Nd方程：
          Matrix<double,MATRIX_SIZE,MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);
          //A乘A的转置是半正定矩阵，因为xT * A * AT * x = (AT * x )T * (AT * x) = AT * x 得出的向量的长度 >= 0!
          matrix_NN = matrix_NN * matrix_NN.transpose();
          Matrix<double,MATRIX_SIZE,1> v_Nd = MatrixXd::Random(MATRIX_SIZE,1);

          clock_t time_stt = clock(); //计时
          //直接求逆
          Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse() * v_Nd ;
          cout << "times of inverse:" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
          // (clock() - time_stt) / (double)CLOCKS_PER_SEC算出来的是秒，乘以1000就是毫秒了
          cout << "x = \n" << x.transpose() << endl;

          //QR分解
          time_stt = clock();
          x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
          cout << "times of QR:" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
          cout << "x = \n" << x.transpose() << endl;

          //正定矩阵可以用cholesky分解
          time_stt = clock();
          x = matrix_NN.ldlt().solve(v_Nd);
          cout << "times of cholesky:" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
          cout << "x = \n" << x.transpose() << endl;
          return 0;
}



