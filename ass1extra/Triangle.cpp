//
// Created by LEI XU on 4/11/19.
//

#include "Triangle.hpp"
#include <algorithm>    // 
#include <array>        // std::array
#include <stdexcept>    // std::runtime_error

Triangle::Triangle()
{
    v[0] << 0, 0, 0;
    v[1] << 0, 0, 0;
    v[2] << 0, 0, 0;

    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

void Triangle::setVertex(int ind, Eigen::Vector3f ver) { v[ind] = ver; }

void Triangle::setNormal(int ind, Vector3f n) { normal[ind] = n; }

void Triangle::setColor(int ind, float r, float g, float b)
{
    if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) ||
        (b > 255.)) {
        throw std::runtime_error("Invalid color values");
    }

    color[ind] = Vector3f((float)r / 255., (float)g / 255., (float)b / 255.);
    return;
}

void Triangle::setTexCoord(int ind, float s, float t)
{
    tex_coords[ind] = Vector2f(s, t);
}

// Q. const在这里的意思？
std::array<Vector4f, 3> Triangle::toVector4() const
{
    std::array<Vector4f, 3> res;
    // std::transform的作用：此处是用一个单变量的函数操作，做的事情是从v的开始到v的结束，
    // 做第4个参数（即lambda表达式）所代表的操作，从res.begin()开始，把结果存入res中
    // 参考 - https://www.cplusplus.com/reference/algorithm/transform/
    // transform()的第4个参数是一个lambda表达式
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) {
        return Vector4f(vec.x(), vec.y(), vec.z(), 1.f);
    });
    return res;
}
