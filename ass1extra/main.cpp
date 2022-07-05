#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 该函数默认绕z轴旋转
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotate_z_180 = Eigen::Matrix4f::Identity();
    rotate_z_180 << -1, 0, 0, 0,
                    0, -1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

    float radian_angle = rotation_angle / 180 * MY_PI;
    model << cos(radian_angle), -1 * sin(radian_angle), 0, 0,
            sin(radian_angle), cos(radian_angle),       0, 0,
            0,                 0,                       1, 0,
            0,                 0,                       0, 1;

    model = model * rotate_z_180;
    return model;
}

// 提高题：得到绕任意过原点的轴的旋转变换矩阵
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    // Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

    // // 思路：先把轴旋转到z轴上，然后做angle角度的旋转，最后再把旋转轴转回去
    // axis.normalize();
    // // 1.
    // Eigen::Matrix4f rotate_to_ori = Eigen::Matrix4f::Identity();
    // rotate_to_ori << 1, 0, 0, 0,
    //                  0, 1, 0, 0,
    //                  axis[0], axis[1], axis[2], 0,
    //                  0, 0, 0, 1;

    // // 2.
    // Eigen::Matrix4f rotation_mat = get_model_matrix(angle);

    // // 3.
    // Eigen::Matrix4f rotate_back = Eigen::Matrix4f::Identity();
    // rotate_back << 1, 0, axis[0], 0,
    //                0, 1, axis[1], 0,
    //                0, 0, axis[2], 0,
    //                0, 0, 0      , 1;

    // rotation = rotate_back * rotation_mat * rotate_to_ori * rotation;
    Eigen::Matrix4f rotate_z_180 = Eigen::Matrix4f::Identity();
    rotate_z_180 << -1, 0, 0, 0,
                    0, -1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

    // Reference: P74 mathematics for 3d game programming and computer graphics
    float radian_angle = angle / 180 * MY_PI;
    float c = cos(radian_angle);
    float s = sin(radian_angle);
    Eigen::Matrix4f axis_rotate = Eigen::Matrix4f::Identity();
    axis_rotate << c + (1 - c) * axis.x(), (1 - c) * axis.x() * axis.y() - s * axis.z(), (1 - c) * axis.x() * axis.z() + s * axis.y(), 0,
                (1 - c) * axis.x() * axis.y() + s * axis.z(), c + (1 - c) * axis.y() * axis.y(), (1 - c) * axis.y() * axis.z() - s * axis.x(), 0,
                (1 - c) * axis.x() * axis.z() - s * axis.y(), (1 - c) * axis.y() * axis.z() + s * axis.x(), c + (1 - c) * axis.z() * axis.z(), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f rotation = axis_rotate * rotate_z_180;
    return rotation;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    zNear = abs(zNear);
    zFar = abs(zFar);
    eye_fov = eye_fov * MY_PI / 180.f;
    float yTop = abs(zNear) * tan(eye_fov / 2);
    float yBottom = -yTop;
    float xRight = aspect_ratio * yTop;
    float xLeft = -xRight;

    // Perspective to orthographic matrix
    Eigen::Matrix4f pers_to_ortho = Eigen::Matrix4f::Identity();
    pers_to_ortho << zNear, 0, 0, 0,
                   0, zNear, 0, 0,
                   0, 0, zNear + zFar, - zNear * zFar,
                   0, 0, 1, 0;

    // Orthographic matrix
    Eigen::Matrix4f ortho_scale = Eigen::Matrix4f::Identity();
    ortho_scale << 2 / (xRight - xLeft), 0, 0, 0,
                  0, 2 / (yTop - yBottom), 0, 0,
                  0, 0, 2 / (zNear - zFar), 0,
                  0, 0, 0, 1;
    Eigen::Matrix4f ortho_translate = Eigen::Matrix4f::Identity();
    ortho_translate << 1, 0, 0, - (xRight + xLeft) / 2,
                      0, 1, 0, - (yTop + yBottom) / 2,
                      0, 0, 1, - (zNear + zFar) / 2,
                      0, 0, 0, 1;
    Eigen::Matrix4f ortho = ortho_scale * ortho_translate;

    projection = ortho * pers_to_ortho * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    // axis为y轴
    Eigen::Vector3f axis = Vector3f::UnitY();
    // Eigen::Vector3f axis = Vector3f::UnitZ();
        // Eigen::Vector3f axis = Vector3f::UnitX();
        // Eigen:: Vector3f axis = Vector3f(1.0f, 1.0f, 0);

    while (key != 27) { // 27代表esc按键。当按下esc时：退出画图的循环并退出程序
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 原代码
        // r.set_model(get_model_matrix(angle));
        
        // 绕axis做angle角度旋转
        r.set_model(get_rotation(axis, angle));

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 5;
        }
        else if (key == 'd') {
            angle -= 5;
        }
    }

    return 0;
}
