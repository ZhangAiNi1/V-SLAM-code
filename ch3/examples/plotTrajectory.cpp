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
#include<unistd.h>              //里面有休眠函数usleep()

using namespace std;
using namespace Eigen;

string trajectory_file = "/home/xtark/V-SLAM code/ch3/examples/trajectory.txt";

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

void DrawTrajectory(vector<Isometry3d,Eigen::aligned_allocator<Isometry3d>> poses)
{
          /*
          * 接下来，我们使用CreateWindowAndBind命令创建了一个视窗对象，
          * 函数的入口的参数依次为视窗的名称、宽度和高度，
          * 该命令类似于OpenCV中的namedWindow，即创建一个用于显示的窗体。
          */
          pangolin::CreateWindowAndBind("Trajectory Viewer",1024,768);
          // 创建名称为“Trajectory Viewer”的GUI窗口，尺寸为1024×768
          glEnable(GL_DEPTH_TEST);
          //启动了深度测试功能，该功能会使得pangolin只会绘制朝向镜头的那一面像素点，避免容易混淆的透视关系出现，因此在任何3D可视化中都应该开启该功能。
          glEnable(GL_BLEND);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            // 创建一个观察相机视图
          pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                /*ProjectMatrix(int w, int h, int fu, int fv, int cu, int cv, int znear, int zfar)
                参数依次为观察相机的图像宽度、高度、4个内参以及最近和最远视距,这个相机并不是SLAM中真实相机
                最小视距是相机可观察到的最近物体距离自己的距离，最大视距是相机可观察到的最远物体距离自己的距离
                可通过鼠标滚轮放大缩小验证。*/
                pangolin::ModelViewLookAt(0,0.1,-0.2,0,0,0,pangolin::AxisNegY)
                
                /*ModelViewLookAt(double x, double y, double z,double lx, double ly, double lz, AxisDirection Up)
                前三个参数是当前相机的坐标，后三个是摄像机的光轴朝向的那个点的坐标（一般是0，0，0点）
                最后一个参数可以规定x轴统一朝上或下，y和z同理
                最后一个参数有两种规定方式，第一种就是利用枚举，第二种就是规定一个向量
                方式一（通过枚举）：
                enum AxisDirection
                {
                AxisNone,
                AxisNegX, AxisX, X轴均朝下或上
                AxisNegY, AxisY, Y轴均朝下或上
                AxisNegZ, AxisZ ，Z轴均朝下或上
                };
                不能将其设置为AxisNone，会报错：what():  'Look' and 'up' vectors cannot be parallel when calling ModelViewLookAt.
                同时不能将其和相机的原点共线，例如pangolin::ModelViewLookAt(0,0.1,0,0,0,0,pangolin::AxisNegY)
                将会报错，修改方式是挪开一点，例如：pangolin::ModelViewLookAt(0,0.1,-0.2,0,0,0,pangolin::AxisNegY)
                方式二（通过向量）
                例如：pangolin::ModelViewLookAt(0.0, 1.0, -1.0, 0, 0, 0,0.0,-1.0,0.0)，只能设置一个轴的朝向，用正负号区分。
                同样的，共线也会报错。*/
          );
          pangolin::View &d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0,1.0,0.0,1.0, -1024.0f/768.0f)
                    .SetHandler(new pangolin::Handler3D(s_cam));
          /*
                    定义地图面板：
                    SetBounds(pangolin::Attach bottom, pangolin::Attach top, pangolin::Attach left, pangolin::Attach right
                                        , double aspect)
                    前四个参数是前四个参数依次表示视图在视窗中的范围（下、上、左、右）
                    ，可以采用相对坐标（0~1）以及绝对坐标（使用Attach对象）。
                    相对坐标：
                              前两个参数设置从0到1，表示从底部到顶部所占的范围
                              后两个参数设置从0到1，表示从左到右所占的范围
                              最后一个参数是设置长宽比。
                    绝对坐标：
                              可使用pangolin::Attach::Pix(pix)，设置需要占据的多少像素例如：
                              SetBounds(0.0,pangolin::Attach::Pix(200),0.0,1.0, -1024.0f/768.0f)
                              表示占据从最下到上方200pix的区域。

                    SetHandler(摄像机把手),设置后可以用键盘鼠标移动摄像机
          */
         
         /***************************************画出轨迹中的所有坐标系*******************************************/
         while(pangolin::ShouldQuit() == false)
         {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                //清空颜色和深度缓存，否则视窗内会保留上一帧的图形
                //不清空深度缓存，界面将是一片黑，如果不清空颜色缓存，界面留下上一针的残影
                d_cam.Activate(s_cam);
                //激活之前设定好的视窗对象
                glClearColor(1.0f,1.0f,1.0f,1.0f);
                //设定背景颜色，四个参数分别是红，绿，蓝，透明度，颜色为0到255映射到0到1之间，本案例用的是白色背景
                glLineWidth(2);
                //设置线宽

                for(size_t i = 0;i < poses.size(); i++)
                {
                        /*在这个循环中绘制poses中存储的的全部坐标系
                        poses存储的变换矩阵是T_WR,
                        下面O_W是相机坐标系原点在世界坐标系下坐标，O_R是相机原点在相机坐标系下的坐标，一般为（0，0，0）
                        故有：O_W = T_WR * O_R   
                        因为等号右侧是一个齐次变换乘法，O_R加一维并赋值1即（0，0，0，1），所以成出来的结果就是t_WR，
                        故O_W = t_WR，所以知道T_WR后即可知道相机在世界坐标系下的位置，故可从T_WR中直接看出相机在何处
                        */
                        Vector3d Ow = poses[i].translation();   //得出t_WR
                        //注意：变换矩阵T在赋值位移向量t时，用的是.pretranslate()，在读取t时用的是.translation()。
                        Vector3d Xw = poses[i] * (0.1 * Vector3d(1,0,0));
                        Vector3d Yw = poses[i] * (0.1 * Vector3d(0,1,0));
                        Vector3d Zw = poses[i] * (0.1 * Vector3d(0,0,1));
                        /*
                                Ow是t_WR，即世界坐标系原点指向相机坐标系原点的向量
                                Xw是在世界坐标系中x轴的单位向量（1，0，0）经过T_WR变换后得到的相机x轴的向量，0.1系数限制向量的长度
                                Yw是在世界坐标系中y轴的单位向量（1，0，0）经过T_WR变换后得到的相机y轴的向量，0.1系数限制向量的长度
                                Zw是在世界坐标系中z轴的单位向量（1，0，0）经过T_WR变换后得到的相机z轴的向量，0.1系数限制向量的长度
                                最终得出一组四个顶点，坐标系原点，x轴终点，y轴终点，z轴终点，后续会将其进行画线连接。
                        */
                       glBegin(GL_LINES);
                        /*
                        glBegin()可以设置作图模式，和glEnd()搭配使用
                        其中GL_LINES是画线段模式，用glVector3d()规定两个顶点后，即可绘制两点间连线
                        在出现多点时，例如下面将有6个顶点，画图规则是两个一组连线，组之间不连线
                        */
                       //画x轴
                       glColor3f(1.0,0.0,0.0);                  
                       //设置画线的颜色，颜色0-255到0-1的映射
                       glVertex3d(Ow[0],Ow[1],Ow[2]);
                       glVertex3d(Xw[0],Xw[1],Xw[2]);
                       //画y轴
                       glColor3f(0.0,1.0,0.0);
                       glVertex3d(Ow[0],Ow[1],Ow[2]);
                       glVertex3d(Yw[0],Yw[1],Yw[2]);
                       //画z轴
                       glColor3f(0.0,0.0,1.0);
                       glVertex3d(Ow[0],Ow[1],Ow[2]);
                       glVertex3d(Zw[0],Zw[1],Zw[2]);
                        glEnd();
                }	

        /***************************************画出轨迹中的所有坐标系之间的连线*******************************************/
        for(size_t i = 0; i< poses.size();i++)
        {
                auto p1 = poses[i];
                auto p2 = poses[i+1];
                glVertex3d(p1.translation()[0],p1.translation()[1],p1.translation()[2]);
                glVertex3d(p2.translation()[0],p2.translation()[1],p2.translation()[2]);
                glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);           //sleep 5ms                    
         }
}
