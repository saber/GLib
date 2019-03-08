/*
 * File: plane_test.cc
 * Project: GLib library
 * Author: gcj
 * Date: 2019/3/8
 * Description: ransac 算法应用程序。
 * License: see the LICENSE.txt file
 */
#include <ransac/ransac_app.h>
#include "Random.h" // 使用 rnadom
#include "eigen_plugins.h" // 修改动态矩阵
#include <ransac/ransac_models.h>

#include <pangolin/pangolin.h> // 基于 opengl 显示库
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <glog/logging.h>
#include <time.h> // 生成随机数用
using namespace glib;

// 绘制所有点,按颜色画出两个平面点以及噪声点
template <typename T>
void DrawPoints(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &data, size_t data_size) {
    glPointSize(5); // 设置点大小！

    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    // 平面 1 点 红色
    for (int i = 0; i < 300; ++i) {
        glVertex3f(float(data(0, i)), float(data(1, i)), float(data(2, i)));
    }
    glEnd();


    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 0.0);

    // 平面 2 点 绿色
    for (int i = 300; i < 500; ++i) {
        glVertex3f(float(data(0, i)), float(data(1, i)), float(data(2, i)));
    }
    glEnd();

    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0);

    // 噪点 蓝色
    for (int i = 500; i < 600; ++i) {
        glVertex3f(float(data(0, i)), float(data(1, i)), float(data(2, i)));
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
    glVertex3d(data(0, idxs[i]), data(1, idxs[i]), data(2, idxs[i]));
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
    pangolin::Var<bool> menuDisplayPlanes("menu.DisplayPlanes", false, true);

    // 随机生成点对（应该改为 标准点加上一个误差为 0 的高斯噪声作为真值点，然后其他点作为纯噪声点）
    const size_t N_planes = 300;
    const size_t N_planes2 = 200;
    const size_t N_noises = 100;
    const double plane_model_param[4] = {1, -1, 1, -2};
    Eigen::Matrix<double, 3, N_planes + N_planes2 + N_noises> data; // 3 x 400
    Random::SeedRand(); // 设置随机种子
    srand((unsigned)time(NULL));
    // 平面有效率点
    for (size_t i = 0; i < N_planes; ++i) {
        const double x = Random::RandomInt(-10, 10);
        const double y = Random::RandomInt(-10, 10);
        const double z = -(plane_model_param[3] + plane_model_param[0] * x + plane_model_param[1] * y) / plane_model_param[2];
        data(0, i) = x;
        data(1, i) = y;
        data(2, i) = z;
        // cout << "(x,y) = " << x << "," << y << endl;
    }

    const double plane_model_param1[4] = {2, 3, 1, -2};

    // 另一个平面点
    for (size_t i = 0; i < N_planes2; ++i) {
        const double x = Random::RandomInt(-5, 10);
        const double y = Random::RandomInt(-5, 10);
        const double z = -(plane_model_param1[3] + plane_model_param1[0] * x + plane_model_param1[1] * y) / plane_model_param1[2];
        data(0, i+N_planes) = x;
        data(1, i+N_planes) = y;
        data(2, i+N_planes) = z;
    }

    // 两个平面共同的的噪点
    for (size_t i = 0; i < N_noises; ++i) {
        const double x = Random::RandomInt(-10, 10);
        const double y = Random::RandomInt(-10, 10);
        const double z = Random::RandomInt(-10, 10);
        data(0, i+N_planes+N_planes2) = x;
        data(1, i+N_planes+N_planes2) = y;
        data(2, i+N_planes+N_planes2) = z;
    }

    // 运行 ransac
    vector<pair<vector<size_t>, T3Dplane> > out_detectedPlanes;
    const double threshold = 0.05;
    const size_t minInliersForValidPlane = 100;

    RansacDetect3DPlane<double>(data, out_detectedPlanes, threshold, minInliersForValidPlane);
    cout << "平面模型数：" << out_detectedPlanes.size() << endl;

    // 打印模型参数
    for (size_t i = 0; i < out_detectedPlanes.size(); ++i) {
        cout << "size = " << out_detectedPlanes[i].first.size()
             << " out_detectedPlanes[i]: ";
        out_detectedPlanes[i].second.model_print();
    }

    // 保存第一个模型的点云为 pcd 文件格式
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    PointCloud::Ptr pointCloud(new PointCloud);
    PointT p;
    vector<size_t> idds = out_detectedPlanes[0].first;
    for (size_t i = 0; i < 300; ++i) {
        p.x = data(0, idds[i]);
        p.y = data(1, idds[i]);
        p.z = data(2, idds[i]);
        p.b = 0;
        p.g = 255;
        p.r = 255;
        pointCloud->points.push_back(p);
    }
    for (size_t i = 300; i < idds.size(); ++i) {
        p.x = data(0, idds[i]);
        p.y = data(1, idds[i]);
        p.z = data(2, idds[i]);
        p.b = 0;
        p.g = 0;
        p.r = 255;
        pointCloud->points.push_back(p);
    }
    cout << "pcl点云大小：" << pointCloud->points.size();
    pcl::io::savePCDFileBinary("plane_model_one.pcd", *pointCloud);


    // 创建面板
    //pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 清除缓冲，方便下次绘图
        d_cam.Activate(s_cam);
        pangolin::glDrawAxis(3); // 创建 3 个轴,可以观看坐标系方向
        glClearColor(1.0f,1.0f,1.0f,1.0f); // 使得 gui 面板显示白色

        if (menuDisplayPlanes) {
            // 依次显示 Ransac 后的数据点
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> after_data = data;
            for (size_t i = 0; i < out_detectedPlanes.size(); ++i) {
                if (i != 0)
                    RemoveEigenDynamicMatrixColumns(after_data, out_detectedPlanes[i-1].first);
                RansacResult<double>(after_data, out_detectedPlanes[i].first, i);//double(i)/out_detectedPlanes.size()
            }
        } else {
            // 绘制所有地图点
            DrawPoints<double>(data, N_planes + N_noises + N_planes2); // 绘制所有地图点
        }

        pangolin::FinishFrame(); // 完成整个绘制结果
    }

    return 0;
}
