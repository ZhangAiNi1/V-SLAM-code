#include<iostream>
#include<fstream>
#include<unistd.h>
#include<sophus/se3.hpp>
#include<pangolin/pangolin.h>

using namespace Sophus;
using namespace std;
using namespace Eigen;

string groundtuth_file = "../groundtruth.txt";
string estimated_file = "../estimated.txt";

typedef vector<Sophus::SE3d,Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

void DrawTrajectory(const TrajectoryType& gt,const TrajectoryType& esti);

TrajectoryType ReadTrajectory(const string& path);

int main(int argc,char** argv)
{
          TrajectoryType groundtuth = ReadTrajectory(groundtuth_file);
          TrajectoryType estimated = ReadTrajectory(estimated_file);
          assert(!groundtuth.empty() && !estimated.empty());          //检测两个容器是否接受到了数据
          assert(groundtuth.size() == estimated.size());                        //检测两个容器中数据数量是否相等
          //assert断言函数，常用于检测程序中错误并终止程序，若形参为false或0,程序终止

          //compute rmse
          double rmse = 0;
          for(int i = 0; i<groundtuth.size() ;i++)
          {
                    Sophus::SE3d T_gt = groundtuth[i];
                    Sophus::SE3d T_esti = estimated[i];
                    double error = (T_gt.inverse() * T_esti).log().norm();
                    rmse += error * error;
          }
          rmse = rmse / groundtuth.size();
          rmse = sqrt(rmse);

          cout << "RMSE = " << rmse << endl;
          DrawTrajectory(groundtuth,estimated);
          return 0;
}

TrajectoryType ReadTrajectory(const string& path)
{
          ifstream fin(path);
          TrajectoryType trajectory;
          if(!fin)
          {
                    cout << "trajectory" << path << "not found" << endl;
                    return trajectory;
          }

          while(!fin.eof())
          {
                    double time,tx,ty,tz,qx,qy,qz,qw;
                    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
                    Sophus::SE3d p1(Quaterniond(qw,qx,qy,qz),Vector3d(tx,ty,tz));
                    trajectory.push_back(p1);
          }
}

void DrawTrajectory(const TrajectoryType& gt,const TrajectoryType& esti)
{
          pangolin::CreateWindowAndBind("Trajectory View",1024,768);
          glEnable(GL_DEPTH_TEST);

          pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                    pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1.0,0.0)
          );

          pangolin::View& d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0,1.0,0.0,1.0,-1024.0f/768.0f)
                    .SetHandler(new pangolin::Handler3D(s_cam));

          while(pangolin::ShouldQuit() == false)
          {
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    d_cam.Activate(s_cam);
                    glClearColor(1.0,1.0,1.0,1.0);
                    glLineWidth(2);

                    for(size_t i = 0;i<gt.size();i++)
                    {
                              glColor3f(1.0,0.0,0.0);
                              glBegin(GL_LINES);
                              auto p1 = gt[i],p2 = gt[i+1];
                              glVertex3d(p1.translation()[0],p1.translation()[1],p1.translation()[2]);
                              glVertex3d(p2.translation()[0],p2.translation()[1],p2.translation()[2]);
                              glEnd();
                    }

                    for(size_t i = 0;i<esti.size();i++)
                    {
                              glColor3f(0.0,0.0,1.0);
                              glBegin(GL_LINES);
                              auto p1 = esti[i],p2 = esti[i+1];
                              glVertex3d(p1.translation()[0],p1.translation()[1],p1.translation()[2]);
                              glVertex3d(p2.translation()[0],p2.translation()[1],p2.translation()[2]);
                              glEnd();
                    }
                    pangolin::FinishFrame();
                    usleep(5000);
          }

}