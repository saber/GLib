#ifndef GLIB_RANSAC_APP_H
#define GLIB_RANSAC_APP_H

#include "ransac_models.h" // 可以进行 ransac 的模型,包含 3d 平面、2d 直线、单应矩阵、F 矩阵

#include <utility> // std::pair
#include <vector>

#include <Eigen/Core>

namespace glib {
using std::pair;
using std::vector;
/*---------------------------------------------------------------
    				RansacDetect3DPlane
---------------------------------------------------------------*/
//! \brief RANSAC 在 3D 点集中，寻找平面模型，必须大于指定的内点数量(minInliersForValidPlane)才算是真正找到了平面模型
//! \param data 包含数据点的所有点云
//! \param out_detectedPlanes RANSAC 找到的多个平面模型
//! \param threshold 数据点到平面模型距离阈值
//! \param minInliersForValidPlane 有效平面模型包含的最小内点个数（根据经验设定）
template <typename NUMTYPE>
void RansacDetect3DPlane(
    Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> data,
    vector<pair<vector<size_t>, T3Dplane> >      &out_detectedPlanes,
    const double                                 threshold,
    const size_t                                 minInliersForValidPlane);

// 模板实例化--声明
#define EXTERN_EXPLICIT_INST_RansacDetect3DPlane(_Type_) \
        extern template void RansacDetect3DPlane<_Type_>(               \
        const Eigen::Matrix<_Type_, Eigen::Dynamic, Eigen::Dynamic> data, \
        vector<pair<vector<size_t>, T3Dplane> >      &out_detectedPlanes, \
        const double                               threshold,           \
        const size_t                               minInliersForValidPlane);

EXTERN_EXPLICIT_INST_RansacDetect3DPlane(float)
EXTERN_EXPLICIT_INST_RansacDetect3DPlane(double)



/*---------------------------------------------------------------
				RansacDetectCircle
 ---------------------------------------------------------------*/
//! \brief 在 2d 点集中寻找圆形模型
//! \param data 所有数据点
//! \param out_detectedCircles RANSAC 找到的多个圆模型
//! \param threshold 点到圆形模型的距离阈值
//! \param minInliersForValidCircle 内点数量至少为这个数，才能算作是模型拟合正确
template <typename NUMTYPE>
void RansacDetectCircle(
    Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> data,
    vector<pair<vector<size_t>, Circle> >      &out_detectedCircles, // 换成内点索引
    const double                               threshold,
    const size_t                               minInliersForValidCircle);

// 模板实例化---声明
#define EXTERN_EXPLICIT_INST_RansacDetectCircle(_Type_) \
    extern template void RansacDetectCircle<_Type_>(    \
        Eigen::Matrix<_Type_, Eigen::Dynamic, Eigen::Dynamic> data, \
        vector<pair<vector<size_t>, Circle> >      &out_detectedCircles, \
        const double                               threshold, \
        const size_t                               minInliersForValidCircle);

EXTERN_EXPLICIT_INST_RansacDetectCircle(float)
EXTERN_EXPLICIT_INST_RansacDetectCircle(double)

} // namespace glib

#endif
