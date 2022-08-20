/*
          读取文件时创建对象：
                    ifstream用于读取文件信息，在创建对象时就可以打开文件，不需要先创建然后再使用open方法。
          判断文件是否正确打开：
                    对ifstream对象取反然后用if语句判断
          判断文件是否读完：
                    用在while判断中用eof()方法
*/
#include<pangolin/pangolin.h>
#include<Eigen/Geometry>

using namespace std;
using namespace Eigen;

string trajectory_file = "/home/xtark/V-slam-book/ch3/examples/trajectory.txt";

//vector第一个参数是数据类型，第二个参数是数据类型使用的分配器类型（因为需要动态分配空间，特殊的数据类型需要特殊的分配器类型）
void DrawTrajectory(vector<Isometry3d,Eigen::aligned_allocator<Isometry3d>>);

int main(int argc,char** argv)
{
          vector<Isometry3d,Eigen::aligned_allocator<Isometry3d>> poses;
          ifstream fin(trajectory_file);
          if(!fin)
          {
                    cout << "can not find trajectory file at" << trajectory_file << endl;
                    return 1;
          }

          while(!fin.eof())
          {
                    double time, tx , ty , tz , qx , qy , qz , qw ;
                    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
                    Isometry3d Twr(Quaterniond(qw,qx,qy,qz));
                    Twr.pretranslate(Vector3d(tx,ty,tz));
                    poses.push_back(Twr);
          }
          cout << "read Toal" << poses.size() << endl;
          cout << "pose entries" << endl;

          DrawTrajectory(poses);
          return 0;

}

void DrawTrajectory(vector<Isometry3d,Eigen::aligned_allocator<Isometry3d>>)
{
          pangolin::CreateWindowAndBind("Trajectory Viewer",1960,1080);
          glEnable(GL_DEPTH_TEST);
          glEnable(GL_BLEND);
          

}
