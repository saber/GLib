/*
 * File: ransac.h
 * Project: GLib library
 * Author: gcj
 * Date: 2019/3/8
 * Description: ransac 实现的基本框架。
 * License: see the LICENSE.txt file
 * reference:  mrpt 相应文件的修改版，下面是 mrpt 库的 license:
 */
 /* +---------------------------------------------------------------------------+
    |                     Mobile Robot Programming Toolkit (MRPT)               |
    |                          http://www.mrpt.org/                             |
    |                                                                           |
    | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
    | See: http://www.mrpt.org/Authors - All rights reserved.                   |
    | Released under BSD License. See details in http://www.mrpt.org/License    |
    +---------------------------------------------------------------------------+ */

#ifndef GLIB_RANSAC_H
#define GLIB_RANSAC_H

#include <iostream>
#include <vector>

#include <Eigen/Core>

//! \brief  一个实现 RANSAC 的基本框架，入口函数为 ExecuteRansac() ，注意在使用过程中需要自己
//!        实现模型拟合函数、数据点到模型距离函数、随机选择的样本点是否退化

namespace glib {

using std::cout;
using std::endl;
using std::vector;

template <typename NUMTYPE = double>
class RANSAC_Template {
public:

    RANSAC_Template() { cout << "ransac constructing." << endl; }
    ~RANSAC_Template() = default;
    //RANSAC_Template(RANSAC& th) = delete; // 不提供拷贝构造函数

    // type declaration

    //! \brief 模型拟合
    //! \param allData: 拟合模型需要的数据集 DxN 矩阵，D 是数据点维度， N 是数据点个数
    //! \param useIndices: s 个随机样本（以索引形式存储，索引对应 allData 矩阵的列）.
    //! \param fitModels: 利用所选 s 个随机样本，计算符合要求的模型（可以有多个不同的模型！比如圆形和直线，给定 3 个样本点都可以拟合处模型）
    typedef void (*RansacFitFunctor)(
        const Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic>    &allData,
        const vector<size_t>                                            &useIndices,
        vector<Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> > &fitModels);

    //! \brief 模型验证
    //! \param allData: 所有数据点集
    //! \param testModels: 对所有数据点进行测试的 Ransac 模型集合
    //! \param distanceThreshold: 判断是内点的距离阈值
    //! \param out_bestModelIndice: 测试当前数据点集符合最好的模型(索引)
    //! \param out_inlierIndices: 内点集(索引)
    typedef void (*RansacDistanceFunctor)(
        const Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic>            &allData,
        const vector<Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> >   &testModels,
        const NUMTYPE                                distanceThreshold,
        unsigned int                                 &out_bestModelIndice,
        vector<size_t>                               &out_inlierIndices);

    //! \brief 验证模型退化
    //! \param allData: 所有数据点集
    //! \param useIndices: s 个随机样本
    //! \return bool: 是否产生退化，0 没有产生退化 1 产生退化
    typedef bool (*RansacDegenerateFunctor)(
        const Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic>   &allData,
        const vector<size_t>                                           &useIndices);

    //! \brief 运行 ransac
    //! \param data
    //! \param fit_func
    //! \param distance_func
    //! \param degenerate_func
    //! \param distance_threshold
    //! \param minimumSizeSampleToFit
    //! \param out_best_model
    //! \param prob_good_sample
    //! \param maxIter
    bool ExecuteRansac(
        const Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> &data,
        RansacFitFunctor             fit_func,
        RansacDistanceFunctor        distance_func,
        RansacDegenerateFunctor      degenerate_func,
        const double                 distance_threshold,
        const unsigned int           minimumSizeSampleToFit,
        vector<size_t>               &out_best_inliers,
        Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic>  &out_best_model,
        const double                 prob_good_sample = 0.999,
        const size_t                 maxIter = 2000) const;

}; // end class RANSAC

typedef RANSAC_Template<double> RANSAC; // 默认 RANSAC 实例为 double 类型

// 外部声明模板实例化---声明
extern template class glib::RANSAC_Template<double>;
extern template class glib::RANSAC_Template<float>;

} // namespace glib

#endif
