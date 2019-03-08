/*
 * File: ransac_app.cc
 * Project: GLib library
 * Author: gcj
 * Date: 2019/3/8
 * Description: ransac app 库。可以在这里添加新的模型 ransac 接口
 * License: see the LICENSE.txt file
 * reference: 关于 3d 平面是参考 mrpt 修改版，下面是 mrpt 库的 license:
 */
 /* +---------------------------------------------------------------------------+
    |                     Mobile Robot Programming Toolkit (MRPT)               |
    |                          http://www.mrpt.org/                             |
    |                                                                           |
    | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
    | See: http://www.mrpt.org/Authors - All rights reserved.                   |
    | Released under BSD License. See details in http://www.mrpt.org/License    |
    +---------------------------------------------------------------------------+ */

#include <ransac/ransac_app.h>
#include <ransac/ransac.h>
#include "eigen_plugins.h"

#include <exception> // 提供异常 catch(exception &)

#include <glog/logging.h>

//! \brief 包含检测 3d 平面，2d 圆的相关函数。包含模型拟合函数、内点到模型距离函数、检测是否退化函数

//! 3D 平面相关函数：包括利用最小随机样本估计模型函数、数据点到模型距离函数、随机样本点是否退化函数
//!                利用最好内点集重新估计对应最好模型模型函数()

namespace glib {

//! \brief 3D 平面相关 ransanc 函数
//! 基本模型拟合函数
template <typename T>
void Ransac_3DplaneFitFunc(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>    &allData,
    const vector<size_t>                                      &useIndices,
    vector<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > &fitModels) {
    DLOG(INFO) << "Ransac_3DplaneFitFunc()";
    CHECK(useIndices.size() == 3) << "最小随机样本数量不符合 3D 平面要求 3 个";

    // 根据索引在数据点集中找到对应的最小样本
    Eigen::Vector3d p1(double(allData(0, useIndices[0])), double(allData(1, useIndices[0])), double(allData(2, useIndices[0])));
    Eigen::Vector3d p2(double(allData(0, useIndices[1])), double(allData(1, useIndices[1])), double(allData(2, useIndices[1])));
    Eigen::Vector3d p3(double(allData(0, useIndices[2])), double(allData(1, useIndices[2])), double(allData(2, useIndices[2])));

    try {
        T3Dplane plane(p1, p2, p3); // 给定 3 点自动计算模型参数
        fitModels.resize(1); // 仅仅拟合一个平面模型
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &M = fitModels[0];
        M.resize(1,4); // 对于动态矩阵设置大小,如果两次改变后，大小不一致，则之前内部元素会丢失
        for (size_t i = 0; i < 4; ++i) {
            M(0, i) = T(plane.coefs_[i]);
        }
    } catch (std::exception &e) {
        LOG(WARNING) << "模型参数计算出现异常!";
        fitModels.clear();
        return ;
    }
}

//! 基本模型距离函数
//! 内部可以实现成寻找多个模型，然后挑选出最好的模型直接返回
template <typename T>
void Ransac_3DplaneDistanceFunc(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>           &allData,
    const vector<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> >  &testModels,
    const T                                distanceThreshold,
    unsigned int                           &out_bestModelIndice,
    vector<size_t>                         &out_inlierIndices) {
    DLOG(INFO) << "Ransac_3DplaneDistanceFunc()";
    CHECK(testModels.size() == 1) << "3D 平面拟合不符合条件(仅仅假定一个模型!)";
    out_bestModelIndice = 0;
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &M = testModels[0];
    CHECK(M.rows() == 1 && M.cols() == 4) << "不符合 3d 平面模型!";

    T3Dplane plane;
    plane.coefs_[0] = M(0, 0);
    plane.coefs_[1] = M(0, 1);
    plane.coefs_[2] = M(0, 2);
    plane.coefs_[3] = M(0, 3);

    const size_t N = allData.cols(); // 点数
    out_inlierIndices.clear();
    out_inlierIndices.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        double dis = plane.Distance(Eigen::Matrix<T,3,1>(allData(0,i), allData(1,i), allData(2,i)));
        if ( dis < distanceThreshold) {
            out_inlierIndices.push_back(i);
        }
    }
}

//! 判断模型退化函数，如果 3 点在一条直线上或者在同一个位置点处 视为退化
//! return ture: 表示 3 个点会退化，无法求解模型参数
template <typename T>
bool Ransac_3DDegenerateFunc(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>  &allData,
    const vector<size_t>                                    &useIndices) {
    DLOG(INFO) << "Ransac_3DDegenerateFunc()";
    CHECK(useIndices.size() == 3) << "最小随机样本数量不符合 3D 平面要求 3 个";

    // 获取 3 个点
    Eigen::Vector3d p1(allData(0, useIndices[0]), allData(1, useIndices[0]), allData(2, useIndices[0]));
    Eigen::Vector3d p2(allData(0, useIndices[1]), allData(1, useIndices[1]), allData(2, useIndices[1]));
    Eigen::Vector3d p3(allData(0, useIndices[2]), allData(1, useIndices[2]), allData(2, useIndices[2]));

    // 判断 3 个点是否在一条直线上
    Eigen::Vector3d p1Top2 = p2 - p1;
    Eigen::Vector3d p1Top3 = p3 - p1;
    if (p1Top2.dot(p1Top3) < std::numeric_limits<double>::epsilon())
        return true;

    // 判断 3 个点是否在一点处
    // 方法：质心到 3 个点的距离不能同时小于 epsilon()
    Eigen::Vector3d centroid(0, 0, 0);
    for (size_t i = 0; i < 3; ++i) {
        centroid(0) += allData(0, useIndices[i]);
        centroid(1) += allData(1, useIndices[i]);
        centroid(2) += allData(2, useIndices[i]);
    }
    centroid /= useIndices.size();
    const double distance1 = Eigen::Vector3d(p1 - centroid).norm();
    const double distance2 = Eigen::Vector3d(p2 - centroid).norm();
    const double distance3 = Eigen::Vector3d(p3 - centroid).norm();
    if (distance1 <= std::numeric_limits<double>::epsilon() &&
        distance2 <= std::numeric_limits<double>::epsilon() &&
        distance3 <= std::numeric_limits<double>::epsilon())
        return true;

    return false;
}

} // namespace glib


/*---------------------------------------------------------------
				RansacDetect3DPlane
 ---------------------------------------------------------------*/
//! note 外部使用这个函数时，必须要在对应的源文件中 extern 声明该函数模板，内部该源文件中使用就不需要
template <typename NUMTYPE>
void glib::RansacDetect3DPlane(
    Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> data,
    vector<pair<vector<size_t>, T3Dplane> >                &out_detectedPlanes, // 换成内点索引
    const double                                           threshold,
    const size_t                                           minInliersForValidPlane) {
    DLOG(INFO) << "RansacDetect3Dplane() start";
    if (data.size() == 0)
        return ;
    out_detectedPlanes.clear();
    vector<size_t> this_best_inliers; // 记录每个模型对应的内点索引

    // 在给定的点集中检测多个平面模型
    for (;;) {
        glib::RANSAC_Template<NUMTYPE> ransac3Dplane;
        Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> this_best_model; //此时为 1x4，但是下面函数中的最好模型不一定是哪个，可以是 H/F
        this_best_inliers.clear();

        ransac3Dplane.ExecuteRansac(data,
                                    Ransac_3DplaneFitFunc,
                                    Ransac_3DplaneDistanceFunc,
                                    Ransac_3DDegenerateFunc,
                                    threshold,
                                    3,
                                    this_best_inliers,
                                    this_best_model,
                                    0.999,
                                    200);
        // 当前检测的模型符合 3d 平面要求
        if (this_best_inliers.size() >= minInliersForValidPlane) {
            DLOG(INFO) << "内点个数：符合要求" << this_best_inliers.size();
            out_detectedPlanes.push_back(
                std::make_pair(
                    this_best_inliers, // 换成内点集
                    T3Dplane(double(this_best_model(0, 0)), double(this_best_model(0, 1)),
                             double(this_best_model(0, 2)), double(this_best_model(0, 3)))
                )
            );

            // 丢弃已经拟合过平面的的点集
            RemoveEigenDynamicMatrixColumns(data, this_best_inliers);
            DLOG(INFO) << "移除多余点后还剩多少列: " << data.cols();
        } else {
            // 没有搜寻到更多的平面模型
            break;
        }

    } // end for
    DLOG(INFO) << "RnasacDetect3Dplane() end";
}

// 模板实例化---定义
#define EXPLICIT_INST_RansacDetect3DPlane(_Type_) \
    template void glib::RansacDetect3DPlane<_Type_>(           \
        Eigen::Matrix<_Type_, Eigen::Dynamic, Eigen::Dynamic> data, \
        vector<std::pair<vector<size_t>, T3Dplane> > &out_detectedPlanes,   \
        const double                               threshold,       \
        const size_t                               minInliersForValidPlane);

EXPLICIT_INST_RansacDetect3DPlane(float)
EXPLICIT_INST_RansacDetect3DPlane(double)

/*---------------------------------------------------------------
				2D 圆相关 ransac 函数
 ---------------------------------------------------------------*/
namespace glib {

template <typename T>
void Ransac_CircleFitFunc(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>    &allData,
    const vector<size_t>                                      &useIndices,
    vector<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > &fitModels) {
    CHECK(allData.rows() == 2) << "给定数据点集必须是二维!";
    CHECK(useIndices.size() == 3) << "给定数据点个数必须是 3";
    DLOG(INFO) << "Ransac_CircleFitFunc()";

    fitModels.resize(1);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &model = fitModels[0]; // 圆有 3 个系数

    Eigen::Vector2d p1(double(allData(0, useIndices[0])), double(allData(1, useIndices[0])));
    Eigen::Vector2d p2(double(allData(0, useIndices[1])), double(allData(1, useIndices[1])));
    Eigen::Vector2d p3(double(allData(0, useIndices[2])), double(allData(1, useIndices[2])));

    try {
        Circle circle_model(p1, p2, p3);
        model.resize(1, 3);
        model(0, 0) = T(circle_model.coefs_[0]);
        model(0, 1) = T(circle_model.coefs_[1]);
        model(0, 2) = T(circle_model.coefs_[2]);
    } catch (std::exception &) {
        LOG(WARNING) << "计算模型失败，产生异常!";
        fitModels.clear();
        return ;
    }
}

template <typename T>
void Ransac_CircleDistanceFunc(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>            &allData,
    const vector<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> >   &testModels,
    const T                                      distanceThreshold,
    unsigned int                                 &out_bestModelIndice,
    vector<size_t>                               &out_inlierIndices) {
    CHECK(allData.rows() == 2) << "给定数据点的维度必须是 2 维!";
    CHECK(testModels.size() == 1) << "拟合圆模型时仅仅一个模型";
    DLOG(INFO) << "Ransac_CircleFitFunc()";

    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>  &model = testModels[0]; // 保证 1x3
    CHECK(model.rows() == 1 && model.cols() == 3) << "不符合圆形模型";
    out_inlierIndices.reserve(allData.cols());
    out_inlierIndices.clear();

    out_bestModelIndice = 0;
    Circle circle;
    circle.coefs_[0] = model(0, 0);
    circle.coefs_[1] = model(0, 1);
    circle.coefs_[2] = model(0, 2);
    for (size_t i = 0; i < size_t(allData.cols()); ++i) {
        const Eigen::Vector2d point(allData(0, i), allData(1, i));
        double dist = abs(circle.Distance(point) - circle.coefs_[2]);
        // cout << "数据点坐标(x,y): " << point(0, 0) << " " << point(1, 0) << endl;
        // cout << "点到圆形距离: " << dist << endl;
        if (dist < distanceThreshold) {
            out_inlierIndices.push_back(i);
        }
    }
}

//! \see ransac_models.h 中对应的圆模型退化情况！
template <typename T>
bool Ransac_CircleDegenerateFunc(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>   &allData,
    const vector<size_t>                                     &useIndices) {
    CHECK(allData.rows() == 2) << "给定数据点的维度必须是 2 维!";
    CHECK(useIndices.size() == 3) << "符合圆模型的数据点个数必须是 3!";
    DLOG(INFO) << "Ransac_CircleDegenerateFunc()";

    Eigen::Vector2d p1(double (allData(0, useIndices[0])), double(allData(1, useIndices[0])));
    Eigen::Vector2d p2(double (allData(0, useIndices[1])), double(allData(1, useIndices[1])));
    Eigen::Vector2d p3(double (allData(0, useIndices[2])), double(allData(1, useIndices[2])));

    const double a = (p1 - p2)(0, 0); // x1 - x2
    const double b = (p1 - p2)(1, 0); // y1 - y2
    const double c = (p1 - p3)(0, 0); // x1 - x3
    const double d = (p1 - p3)(1, 0); // y1 - y3

    // 三点共线或者三点距离很近(共点)视为退化！
    if (a*d == b*c)
        return true;

    // 判断 3 个点是否在一点处
    // 方法：质心到 3 个点的距离不能同时小于 epsilon()
    Eigen::Vector2d centroid(0, 0);
    for (size_t i = 0; i < 2; ++i) {
        centroid(0) += allData(0, useIndices[i]);
        centroid(1) += allData(1, useIndices[i]);
    }
    centroid /= useIndices.size();
    const double distance1 = Eigen::Vector2d(p1 - centroid).norm();
    const double distance2 = Eigen::Vector2d(p2 - centroid).norm();
    const double distance3 = Eigen::Vector2d(p3 - centroid).norm();

    if (distance1 <= std::numeric_limits<double>::epsilon() &&
        distance2 <= std::numeric_limits<double>::epsilon() &&
        distance3 <= std::numeric_limits<double>::epsilon())
        return true;
    return false;
}

} // namespace glib
/*---------------------------------------------------------------
				RansacDetectCircle
 ---------------------------------------------------------------*/
template <typename NUMTYPE>
void glib::RansacDetectCircle(
    Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> data,
    vector<pair<vector<size_t>, Circle> >      &out_detectedCircles, // 换成内点索引
    const double                               threshold,
    const size_t                               minInliersForValidCircle) {
    CHECK(data.rows() == 2) << "数据点维度是 2!";
    DLOG(INFO) << "RansacDetectCircle() start";

    vector<size_t> this_best_inliers;
    RANSAC_Template<NUMTYPE> ransac;

    // 拟合多个圆形模型!
    for (;;) {
        Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> this_best_model;
        this_best_inliers.clear(); // 在下面 ExecuteRansac 函数中会清零。这里可以不清理
        ransac.ExecuteRansac(
                            data,
                            Ransac_CircleFitFunc,
                            Ransac_CircleDistanceFunc,
                            Ransac_CircleDegenerateFunc,
                            threshold,
                            3,
                            this_best_inliers,
                            this_best_model,
                            0.999,
                            200
                            );
        if (this_best_inliers.size() > minInliersForValidCircle) {
            DLOG(INFO) << "内点个数：符合要求" << this_best_inliers.size();
            Circle circle;
            circle.coefs_[0] = double(this_best_model(0, 0));
            circle.coefs_[1] = double(this_best_model(0, 1));
            circle.coefs_[2] = double(this_best_model(0, 2));
            out_detectedCircles.push_back(std::make_pair(this_best_inliers, circle));

            // 丢弃已经拟合过平面的点集
            RemoveEigenDynamicMatrixColumns(data, this_best_inliers);
            DLOG(INFO) << "移除多余点后还剩多少列: " << data.cols();
        } else {
            break;
        }
    }
    DLOG(INFO) << "RansacDetectCircle() end";
}

// 模板实例化---定义
#define EXPLICIT_INST_RansacDetectCircle(_Type_) \
    template void glib::RansacDetectCircle<_Type_>( \
        Eigen::Matrix<_Type_, Eigen::Dynamic, Eigen::Dynamic> data, \
        vector<pair<vector<size_t>, Circle> >      &out_detectedCircles, \
        const double                               threshold, \
        const size_t                               minInliersForValidCircle);
EXPLICIT_INST_RansacDetectCircle(float)
EXPLICIT_INST_RansacDetectCircle(double)
