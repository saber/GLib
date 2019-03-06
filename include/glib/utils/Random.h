/*
*This file is part of GLib.
*This file is a modified version of DLib/include/Random.h, see FreeBSD license below.
*/

/*
 * File: Random.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: April 2010, November 2011
 * Description: manages pseudo-random numbers
 * License: see the LICENSE.txt file
 *
 */

#ifndef GLIB_RANDOM_H_
#define GLIB_RANDOM_H_

#include <cstdlib> // rand() srand()
#include <vector>

namespace glib{

//! \brief 生成伪随机数
//! \see https://blog.csdn.net/candyliuxj/article/details/4396666
class Random
{
public:
  class UnrepeatedRandomizer;

public:
    //! \breif 设置当前时间为随机种子
    static void SeedRand();

    //! \brief 仅仅设置一次种子
    static void SeedRandOnce();

    //! \brief 设置种子为给定值
    static void SeedRand(int seed);

    //! \brief 设置种子为给定值仅一次
    static void SeedRandOnce(int seed);

    //! \brief 返回 [0,1] 之间的随机数
    template <class T>
    static T RandomValue() {
    	return (T)rand()/(T)RAND_MAX;
    }

    //! \brief 返回 [min, max] 之间随机数(可以指定随机数类型!)
    template <class T>
    static T RandomValue(T min, T max) {
    	return Random::RandomValue<T>() * (max - min) + min;
    }

    //! \brief 返回 [min, max] 之间的 int 类型的随机数
    static int RandomInt(int min, int max);

    //! \brief 根据 Box-Muller 方法，计算正态分布随机数
    template <class T>
    static T RandomGaussianValue(T mean, T sigma) {
        // Box-Muller transformation
        T x1, x2, w, y1;

        do {
          x1 = (T)2. * RandomValue<T>() - (T)1.;
          x2 = (T)2. * RandomValue<T>() - (T)1.;
          w = x1 * x1 + x2 * x2;
        } while ( w >= (T)1. || w == (T)0. );

        w = sqrt( ((T)-2.0 * log( w ) ) / w );
        y1 = x1 * w;

        return( mean + y1 * sigma );
    }

    //! \brief 利用 random 库包含的正态分布随机数
    //! \see https://blog.csdn.net/caroline_wendy/article/details/17335655
    //! \detail
    //! #include <ctime> #include <random>
    //!  std::default_random_engine e(time(NULL))
    //!  std::normal_distribution<double> normal_distri(mean, sigma)
    //!  normal_distri(e) 返回正态分布随机数


    //! \brief 随机填满 vec 内部值
    //! \note vec 在使用之间必须要 resize() 指定大小
    //! \param vec 保存随机值
    static void RandomVector(std::vector<size_t> &vec,
                             size_t min, size_t max);
private:

    // If SeedRandOnce() or SeedRandOnce(int) have already been called
    static bool already_seeded_;

};

// ---------------------------------------------------------------------------

//! \brief 不带有重复数字的伪随机数[min, max]
class Random::UnrepeatedRandomizer
{
public:
    UnrepeatedRandomizer(int min, int max);
    ~UnrepeatedRandomizer(){}

    UnrepeatedRandomizer(const UnrepeatedRandomizer& rnd);

    UnrepeatedRandomizer& operator=(const UnrepeatedRandomizer& rnd);

    //! \brief 返回[min,max]之间的一个不重复的随机数，如果范围[min, max]数字已经全部都返回过一次，
    //!       此时会从新在[min, max] 之间随机返回一个数字。效果就是周期性的返回范围内的数字
    int get();

    //! \brief 判断存储[min, max]值是否为空
    inline bool empty() const { return values_.empty(); }

    //! \brief 返回当前存储[min, max]值的 vector 还包含几个元素
    inline unsigned int left() const { return values_.size(); }

    //! \brief 重置 values_ 存储值为[min, max]
    void reset();

protected:

    //! \brief 创建[min, max]随机不重复数字序列
    void createValues();

protected:

    /// Min of range of values
    int min_;
    /// Max of range of values
    int max_;

    /// Available values
    std::vector<int> values_;

};

}

#endif
