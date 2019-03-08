/*
 * File: eigen_plugins.h
 * Project: GLib library
 * Author: gcj
 * Date: 2019/3/8
 * Description: Eigen 库的一些应用插件。
 * License: see the LICENSE.txt file
 * reference: 参考 mrpt 中的 eigen_plugins.h。做了一些改动。下面是 mrpt 库的 license:
 */
 /* +---------------------------------------------------------------------------+
    |                     Mobile Robot Programming Toolkit (MRPT)               |
    |                          http://www.mrpt.org/                             |
    |                                                                           |
    | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
    | See: http://www.mrpt.org/Authors - All rights reserved.                   |
    | Released under BSD License. See details in http://www.mrpt.org/License    |
    +---------------------------------------------------------------------------+ */

#ifndef GLIB_EIGEN_PLUGINS_H
#define GLIB_EIGEN_PLUGINS_H

#include <iostream>
#include <vector>

#include <Eigen/Core>

// Eigen 插件
namespace glib {

//! \brief 移除矩阵某列元素，假定元素索引已经按照升序排列，且内部索引无重复！
template <typename T>
void UnsafeRemoveColumns(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &data,
                         const std::vector<size_t> &idxsToRemove) {
    size_t k = 1; // 记录已经移除元素的个数

    // 从后向前依次向前移动
    for (std::vector<size_t>::const_reverse_iterator it = idxsToRemove.rbegin();
         it != idxsToRemove.rend(); ++it, ++k) {
        const size_t nC = data.cols() - *it - k;
        if (nC > 0) {
            data.block(0, *it, data.rows(), nC)
            = data.block(0, *it + 1, data.rows(), nC).eval();
        }
    }

    data.conservativeResize(Eigen::NoChange, data.cols() - idxsToRemove.size()); //此时会保留元素的值 与 resize() 不同
}

//! \brief 移除 Eigen 动态矩阵某列元素
template <typename T>
void RemoveEigenDynamicMatrixColumns(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &data,
                                     const std::vector<size_t> &idxsToRemove) {
    std::vector<size_t> idxs = idxsToRemove;
    std::sort(idxs.begin(), idxs.end()); // 按照升序排序，方便后面删除指定列

    // 在 idxs 中将相邻相同元素取出放在后面排列, 0 1 1 2 2 经过下面函数后，就会变为 0 1 2 1 2
    // 然后返回值 itEnd 指向后面那个 1 这个元素
    std::vector<size_t>::iterator itEnd = std::unique(idxs.begin(), idxs.end());
    idxs.resize(itEnd - idxs.begin()); // 调整不相同的元素！保证删除不重复的列

    UnsafeRemoveColumns(data, idxs);
}

} // namespace glib

#endif
