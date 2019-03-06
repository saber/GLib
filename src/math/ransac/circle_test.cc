#include "ransac_app.h" // 使用 ransac 拟合圆形模型函数
#include "Random.h"
#include "eigen_plugins.h" // 修改动态矩阵
#include "ransac_models.h" // 使用圆形模型

#include <random> // 产生高斯噪声分布
#include <pangolin/pangolin.h> // 基于 opengl 显示库
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <glog/logging.h>
#include <time.h> // 生成随机数用
using namespace glib;

// 绘制所有点,按颜色
template <typename T>
void DrawPoints(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &data, size_t data_size) {
    glPointSize(5); // 设置点大小！

    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    // 圆 1 点 红色
    for (int i = 0; i < 300; ++i) {
        glVertex2f(float(data(0, i)), float(data(1, i)));
    }
    glEnd();

    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 0.0);

    // 圆 2 点 绿色
    for (int i = 300; i < 500; ++i) {
        glVertex2f(float(data(0, i)), float(data(1, i)));
    }
    glEnd();

    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0);

    // 噪点 蓝色
    for (int i = 500; i < 600; ++i) {
        glVertex2f(float(data(0, i)), float(data(1, i)));
    }
    glEnd();
}

// 根据索引显示指定数据点
template <typename T>
void RansacResult(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &data,
                  const vector<size_t> &idxs, double rgb) {
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(rgb, 0, 1);
    for (size_t i = 0; i < idxs.size(); ++i) {
      glVertex2d(data(0, idxs[i]), data(1, idxs[i]));
    }
    glEnd();
}

int main(int argc, char **argv) {
    // pangolin 显示
    pangolin::CreateWindowAndBind("Pangolin", 2014, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND); // 使用颜色混合
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // 使用颜色混合选项，与上面配对使用
    // 摆放一个相机？
    pangolin::OpenGlRenderState s_cam(
                                 // 窗口大小    相机内参数          近点 远点
        pangolin::ProjectionMatrix(1024, 768, 420, 420, 320, 240, 0.2, 100), // 透视矩阵，
        // 前三个参数：相机在世界坐标的位置 中间三个：相机镜头对准的物体在世界坐标的位置   后三个参数：相机头顶朝向的方向
        // 参考：https://blog.csdn.net/ivan_ljf/article/details/8764737
        pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, pangolin::AxisNegY)
        // pangolin::ModelViewLookAt(0, -0.7, -0.8, 0, 0, 0, pangolin::AxisNegY)
    );

    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay() // 表示相机观察的平面
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
        .SetHandler(&handler);

    // 创建一个 GUI 面板
    // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));

    // 建立计数变量，记录平面模型有几个,显示全部平面！
    pangolin::Var<bool> menuDisplayCircles("menu.DisplayCircles", false, true);

    // 随机生成点对（应该改为 标准点加上一个误差为 0 的高斯噪声作为真值点，然后其他点作为纯噪声点）
    const size_t N_circle_one = 300;
    const size_t N_circle_two = 200;
    const size_t N_noises = 100;

    Eigen::Matrix<double, 2, N_circle_one + N_circle_two + N_noises> data; // 2 x 400
    Random::SeedRand(); // 设置随机种子
    srand((unsigned)time(NULL));
    std::default_random_engine engine(time(NULL)); // 设置随机数引擎
    std::normal_distribution<double> rand_num(5, 0.25); // sigama = 0.5 ，因此 ransac 阈值为 sqrt(3.84 * 0.5^2) = sqrt(0.96)

    // 圆1，根据圆的参数方程产生。圆心为 (0,0),半径为 r = 5 的圆形，噪声误差服从高斯分布(r,0.25)
    for (size_t i = 0; i < N_circle_one; ++i) {
        const double r = rand_num(engine); // 根据高斯分布产生随机数
        const int theta = Random::RandomInt(0, 360); // 产生一个 [0, 360]° 角度随机数

        const double x = r * cos(theta);
        const double y = r * sin(theta);
        data(0, i) = x;
        data(1, i) = y;
        // cout << "(x,y) = " << x << "," << y << endl;
    }

    // 圆2 噪声误差仍然服从上面的分布，半径仍然是 5, 圆心是 (2,2)
    for (size_t i = 0; i < N_circle_two; ++i) {
        const double r = rand_num(engine); // 根据高斯分布产生随机数
        const int theta = Random::RandomInt(0, 360); // 产生一个 [0, 360]° 角度随机数

        const double x = 2 + r * cos(theta); // 圆心为(2,2)
        const double y = 2 + r * sin(theta);
        data(0, i+N_circle_one) = x;
        data(1, i+N_circle_one) = y;
    }

    // 两个圆共同的的噪点，【-5,7】
    for (size_t i = 0; i < N_noises; ++i) {
        const double x = Random::RandomInt(-5, 7);
        const double y = Random::RandomInt(-5, 7);
        data(0, i+N_circle_one+N_circle_two) = x;
        data(1, i+N_circle_one+N_circle_two) = y;
    }

    // 运行 ransac
    vector<pair<vector<size_t>, Circle> > out_detectedCircles; // 圆的模型集
    const double threshold = 0.9; // 根据卡方分布计算 0.97
    const size_t minInliersForValidPlane = 50;

    RansacDetectCircle<double>(data, out_detectedCircles, threshold, minInliersForValidPlane);
    cout << "拟合模型数：" << out_detectedCircles.size() << endl;

    // 根据拟合出来的平面模型，分别生成对应的平面点
    for (size_t i = 0; i < out_detectedCircles.size(); ++i) {
        cout << "size = " << out_detectedCircles[i].first.size()
             << " out_detectedCircles[i]: ";
        out_detectedCircles[i].second.model_print();
    }

    // 保存数据点为 pcl 格式
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    PointCloud::Ptr pointCloud(new PointCloud);
    PointT p;
    vector<size_t> idds = out_detectedCircles[0].first;
    for (size_t i = 0; i < N_circle_one; ++i) {
        p.x = data(0, idds[i]);
        p.y = data(1, idds[i]);
        p.z = 0;
        p.b = 0;
        p.g = 255;
        p.r = 255;
        pointCloud->points.push_back(p);
    }
    for (size_t i = N_circle_one; i < idds.size(); ++i) {
        p.x = data(0, idds[i]);
        p.y = data(1, idds[i]);
        p.z = 0;
        p.b = 0;
        p.g = 0;
        p.r = 255;
        pointCloud->points.push_back(p);
    }
    cout << "pcl 点云大小：" << pointCloud->points.size();
    pcl::io::savePCDFileBinary("circle_model_one.pcd", *pointCloud);


    // 创建面板
    //pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 清除缓冲，方便下次绘图
        d_cam.Activate(s_cam);
        pangolin::glDrawAxis(3); // 创建 2 个轴,可以观看坐标系方向
        glClearColor(1.0f,1.0f,1.0f,1.0f); // 使得 gui 面板显示白色

        if (menuDisplayCircles) {
            //依次显示 Ransac 后的数据点
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> after_data = data;
            for (size_t i = 0; i < out_detectedCircles.size(); ++i) {
                if (i != 0)
                    RemoveEigenDynamicMatrixColumns(after_data, out_detectedCircles[i-1].first);
                RansacResult<double>(after_data, out_detectedCircles[i].first, i);//double(i)/out_detectedCircles.size()
            }
        } else {
            // 绘制所有地图点,按颜色
            DrawPoints<double>(data, N_circle_one + N_circle_two + N_noises); // 绘制所有地图点
        }

        pangolin::FinishFrame(); // 完成整个绘制结果
    }

    return 0;
}
