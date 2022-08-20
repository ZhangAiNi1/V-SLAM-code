/*
          Isometry在构造的时候可以直接赋值四元数。
*/

#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>

using namespace Eigen;
using namespace std;

int main(int atranslatergc,char** argv)
{
          Quaterniond q1(0.35,0.2,0.3,0.1);
          Quaterniond q2(-0.5,0.4,-0.1,0.2);
          q1.normalize();
          q2.normalize();
 //         Isometry3d TR1_W(q1);
          Isometry3d TR1_W = Isometry3d::Identity();
          TR1_W.rotate(q1);          
          TR1_W.pretranslate(Vector3d(0.3,0.1,0.1));

         Isometry3d TR2_W(q2);
          TR2_W.pretranslate(Vector3d(-0.1,0.5,0.3));
          Vector3d pR1(0.5,0,0.2);

          Vector3d pR2 = TR2_W * TR1_W.inverse() * pR1;
          cout << "pR1在萝卜二号下的坐标是: \n" << pR2.transpose() << endl;

          return 0;

}