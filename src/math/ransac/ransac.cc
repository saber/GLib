
#include "ransac.h"
#include "format.h"
#include "Random.h"

#include <cstdio>
#include <math.h>

#include <string>

#include <glog/logging.h> // CHECK()、LOG()...

namespace glib {
/*---------------------------------------------------------------
			ransac generic implementation RANSAC 实现框架！
 ---------------------------------------------------------------*/
//! \note 这个函数默认返回一个最好的模型。
template <typename NUMTYPE>
bool RANSAC_Template<NUMTYPE>::ExecuteRansac(
    const Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic>    &data,
    RansacFitFunctor             fit_func,
    RansacDistanceFunctor        distance_func,
    RansacDegenerateFunctor      degenerate_func,
    const double                 distance_threshold,
    const unsigned int           minimumSizeSampleToFit,
    vector<size_t>               &out_best_inliers,
    Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic>          &out_best_model,
    const double                 prob_good_sample,
    const size_t                 maxIter) const {
    CHECK(prob_good_sample < 1 && prob_good_sample > 0)
        << "至少一次没有野值的概率必须属于(0,1)";
    CHECK(maxIter) << "最大迭代次数 >= 1";
    CHECK(minimumSizeSampleToFit >= 1) << "随机样本个数至少为 1";
    LOG(INFO) << "ExecuteRansac() start!";

    const size_t dataDimension = data.rows(); // 数据维度函数
    const size_t dataPointsNum = data.cols(); // 数据点个数
    CHECK(dataDimension >= 1) << "数据维度 >= 1";
    CHECK(dataPointsNum >= 1) << "数据点个数 >= 1";

    const size_t maxDegenerateTrials = 100; // 最大退化次数（可修改！）
    out_best_inliers.clear();

    size_t trial_count = 0;
    size_t best_score = std::string::npos;
    size_t N = 1; // 预先初始化试验次数，后面进行更改！

    vector<size_t> samples; // indices  s 个随机样本
    samples.resize(minimumSizeSampleToFit);
    Random::SeedRand(); // 设置随机种子

    while (N > trial_count) {
        // 每次试验，随机选择 s 个数据样本，估计模型 M
        // 需要注意 s 个数据样本不能退化
        bool isDegenerate = true; // 默认产生退化现象
        size_t degenerate_trial = 1; // 退化次数
        vector<Eigen::Matrix<NUMTYPE, Eigen::Dynamic, Eigen::Dynamic> > models; // 数据拟合出来的模型

        while (isDegenerate) {

            // 随机选择 s 个数据样本 samples 使用之前需要 resize() 调整到指定随机样本大小
            Random::RandomVector(samples, 0, dataPointsNum-1);

            // 没有产生退化现象，需要拟合模型
            if (!degenerate_func(data, samples)) {
                fit_func(data, samples, models); // models 可以代表拟合一系列的模型
                isDegenerate = models.empty();
            }

            if (++degenerate_trial > maxDegenerateTrials) {
                LOG(WARNING) << "不能选择一个非退化的随机样本！";
                //cout << "不能选择一个非退化的随机样本！" << endl;
                break;
            }
        } // end while(isDegenerate)

        unsigned int bestModelIndice = (models.size() > 0 ? models.size() : 0); // 初始化一个非要求索引范围的值，可以取 1000 之类的值
        vector<size_t> inlierIndices;

        // 进行内点筛选
        if (!isDegenerate) {
            // 根据距离函数，从众多模型中选出一个最好的模型及其内点
            distance_func(data, models, NUMTYPE(distance_threshold), bestModelIndice, inlierIndices);
            CHECK(bestModelIndice < models.size()) << "超出模型索引范围";
        }

        const size_t ninliers = inlierIndices.size(); // 内点数量
        bool isUpdateN = (trial_count == 0); // 第一次实验时要更新总试验次数 N

        // 更新最好得分以及最好模型，确定更新总试验次数
        if (ninliers > best_score || (best_score == std::string::npos && ninliers != 0)) {
            best_score = ninliers; // 对于 H/F 矩阵这里可以转换成误差

            out_best_inliers = inlierIndices;
            out_best_model = models[bestModelIndice];
            isUpdateN = true; // 每次获得最好得分时，要随时更新总试验次数！
        }

        // 更新最大试验次数 N
        if (isUpdateN) {
            // N = log(1-p)/log(1-(1-ci)^s) ; 其中 ci : 野值的比例， s: 随机样本个数
            double inlierRatio = ninliers/static_cast<double>(dataPointsNum);
            double s_noWildRatio = 1 - pow(inlierRatio, static_cast<double>(minimumSizeSampleToFit));

            // 判断计算出来的比率是否超过 (0,1) 范围，之后进行调整！
            s_noWildRatio = std::max(std::numeric_limits<double>::epsilon(), s_noWildRatio); // 避免后面 log(s_noWildRatio) = -Inf
            s_noWildRatio = std::min(1.0 - std::numeric_limits<double>::epsilon(), s_noWildRatio); // 避免后面 log(s_noWildRatio) = 0

            N = static_cast<size_t>(log(1-prob_good_sample)/log(s_noWildRatio));
            DLOG(INFO) << format("迭代次数: #%u 估计的迭代总数: %u   s个点无野值概率: %f  #内点数: %u\n",
                                (unsigned)trial_count ,(unsigned)N, s_noWildRatio, (unsigned)ninliers);
        }

        ++trial_count;
        DLOG(INFO) << format("当前迭代: %u 迭代上限: %u \r",(unsigned int)trial_count, (unsigned int)ceil(static_cast<double>(N)));

        // 不能超过自己指定的最大的迭代次数
        if (trial_count > maxIter) {
            LOG(WARNING ) << format("最大试验次数已经达到 (%u)\n", (unsigned)maxIter );
            break;
        }
    } // end while(N > trial_count)

    if (out_best_inliers.size() > 0) {
        // TODO 从新利用所有内点集，拟合一个模型！然后作为最终的最好模型
        LOG(INFO) << "ExecuteRansac() success !";
        return true;
    } else {
        LOG(WARNING) << "RANSAC 迭代失败，没有找到最好的模型!";
        return false;
    }
}

} // namespace glib

// 模板实例化---定义
template class glib::RANSAC_Template<double>;
template class glib::RANSAC_Template<float>;
