#ifndef GLIB_RANSAC_MODELS_H
#define GLIB_RANSAC_MODELS_H

#include <iostream>
#include <math.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry> // 调用向量叉积 cross 函数

//! \brief 将要拟合的模型。包括 3d 平面、2d 圆模型
namespace glib {
using std::vector;
using std::cout;
using std::endl;

//! \brief 3d 平面 ：方程一般形式：ax + by + cz + d = 0
//! \note 退化情况默认没有考虑
class T3Dplane {
public:
    // 两种初始化方式：1）三点确定一个平面 2）四个系数确定平面模型
    // construct func
    T3Dplane() = default;
    T3Dplane(Eigen::Vector3d &point1, Eigen::Vector3d &point2,
             Eigen::Vector3d &point3) {
        ComputeModelCoefs(point1, point2, point3);
    }
    // 模型参数
    T3Dplane(double a, double b, double c, double d) {
        coefs_[0] = a;
        coefs_[1] = b;
        coefs_[2] = c;
        coefs_[3] = d;
    }
    T3Dplane(const T3Dplane &obj) = default;
    T3Dplane& operator=(const T3Dplane &obj) = default;
    ~T3Dplane() = default;

    //! \brief 点到平面距离
    template <typename T>
    T Distance(const Eigen::Matrix<T, 3, 1> &point) {
        //cout << "enter: " << point(0,0) << " " << point(1,0) << " " << point(2,0) << endl;
        return T((abs(coefs_[0]*point(0,0) + coefs_[1]*point(1,0) + coefs_[2]*point(2,0) + coefs_[3]))) \
            / sqrt(pow(coefs_[0], 2) + pow(coefs_[1], 2) + pow(coefs_[2], 2));
    }

    template <typename T>
    T Distance(const T &x, const T &y, const T &z) {
        return T((abs(coefs_[0]*x + coefs_[1]*y + coefs_[2]*z + coefs_[3]))) \
            / sqrt(pow(coefs_[0], 2) + pow(coefs_[1], 2) + pow(coefs_[2], 2));
    }

    //! \brief 输出平面模型系数
    void model_print() {
        for (int i = 0; i < 4; ++i) {
            cout << coefs_[i] << " ";
        }
        cout << endl;
    }

    double coefs_[4] = {0, 0, 0, 0}; // 平面模型的 4 个系数

protected:

    //! \brief 根据 3 个点，计算平面模型。未考虑退化现象
    //! \see https://blog.csdn.net/newproblems/article/details/77651517
    void ComputeModelCoefs(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                           const Eigen::Vector3d &p3) {
        Eigen::Vector3d abc = (p2 - p1).cross((p3 - p1));
        coefs_[0] = abc(0);
        coefs_[1] = abc(1);
        coefs_[2] = abc(2);
        coefs_[3] = -abc.dot(p1);
    }

};

//! \brief 圆的标准方程: (x-x0)^2 + (y-y0)^2 = r^2。
// 圆的 3 个参数 x0, y0, r
class Circle {
public:
    Circle () = default;
    Circle (double x0, double y0, double r) {
        coefs_[0] = x0;
        coefs_[1] = y0;
        coefs_[2] = r ;
    }
    Circle (Eigen::Vector2d &p1, Eigen::Vector2d &p2, Eigen::Vector2d &p3) {
        // 根据三点计算圆模型
        ComputeModelCoefs(p1, p2, p3);
    }
    Circle(const Circle &obj) = default;
    Circle& operator=(const Circle &obj) = default;
    ~Circle() = default;

    //! \brief 根据 3 点计算圆模型，假定处理非退化情况。
    //! \detail
    // 数学推倒，将 3 点代入标准圆模型
    //    (x1-x0)^2 + (y1-y0)^2 = r^2 ----(1)
    //    (x2-x0)^2 + (y2-y0)^2 = r^2 ----(2)
    //    (x3-x0)^2 + (y3-y0)^2 = r^2 ----(3)
    // (1) 式分别减去 (2) 式和 (3) 式得到如下方程:
    //    x0(x1-x2) + y0(y1-y2) = (x1^2 - x2^2)/2 + (y1^2 - y2^2)/2
    //    x0(x1-x3) + y0(y1-y3) = (x1^2 - x3^2)/2 + (y1^2 - y3^2)/2
    // 写成矩阵形式:
    //    [x1-x2 y1-y2] [x0] = [ (x1^2-x2^2)/2 + (y1^2-y2^2)/2 ]
    //    [x1-x3 y1-y3] [y0]   [ (x1^2-x3^2)/2 + (y1^2-y3^2)/2 ]
    //                       == [ m ]
    //                          [ n ]
    // 因此可得退化情况是上面矩阵方程无解，即：
    // (x1-x2)(y1-y3) = (y1-y2)(x1-x3) ==> (x1-x2)/(x1-x3) = (y1-y2)/(y1-y3) 三点共线
    // 由上面公式可以协成如下函数
    void ComputeModelCoefs(const Eigen::Vector2d &p1,
                           const Eigen::Vector2d &p2,
                           const Eigen::Vector2d &p3) {
        const double a = (p1 - p2)(0, 0); // x1 - x2
        const double b = (p1 - p2)(1, 0); // y1 - y2
        const double c = (p1 - p3)(0, 0); // x1 - x3
        const double d = (p1 - p3)(1, 0); // y1 - y3
        assert(a * d != b * c); // 不能产生退化现象

        const double e = (p1 + p2)(0, 0); // x1 + x2
        const double f = (p1 + p2)(1, 0); // y1 + y2
        const double g = (p1 + p3)(0, 0); // x1 + x3
        const double h = (p1 + p3)(1, 0); // y1 + y3
        const double m = (a * e + b * f)/2.0;
        const double n = (c * g + d * h)/2.0;
        const double det = a * d - b * c;
        coefs_[0] = (m * d - b * n)/det;
        coefs_[1] = (a * n - m * c)/det;
        coefs_[2] = sqrt(pow(p1(0, 0) - coefs_[0], 2) + pow(p1(1, 0) - coefs_[1], 2));
    }

    //! \brief 点到圆心距离的平方,但是这里要返回的是，点到圆的边的距离，因为这个距离满足高斯分布!
    template <typename T>
    T Distance(const Eigen::Matrix<T, 2, 1> &point) {
        return T(sqrt(pow(point(0, 0) - coefs_[0], 2) + pow(point(1, 0) - coefs_[1], 2)));
    }

    template <typename T>
    T Distance(const T &x, const T &y) {
        return T(sqrt(pow(x - coefs_[0], 2) + pow(y - coefs_[1], 2)));
    }

    //! \brief 输出平面模型系数
    void model_print() {
        for (int i = 0; i < 3; ++i) {
            cout << coefs_[i] << " ";
        }
        cout << endl;
    }

    double coefs_[3] = {0, 0, 0}; // 默认圆的 3 个参数顺序为 x0, y0, r

};

} // namespace glib

#endif
