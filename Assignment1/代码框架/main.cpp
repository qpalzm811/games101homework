//参考 https://blog.csdn.net/ycrsw/article/details/123834579
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
    translate << 
        1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float pi = acos(-1);
    float radian = rotation_angle / 180 * pi;//切记转弧度

    Eigen::Matrix4f rotationZ;
    rotationZ << 
        cos(radian), -sin(radian), 0, 1,
        sin(radian),cos(radian), 0, 1,
        0, 0, 1, 1,
        0, 0, 0, 1;

    model = rotationZ * model;
     
    return model;
}
// 任意轴旋转
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    double radian = angle / 180 * MY_PI;
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    //用到罗德里格旋转公式 Rodrigues' rotation formula

    Eigen::Vector4f axi;
    axi << axis.x(), axis.y(), axis.z(), 0;
    Eigen::RowVector4f taxi;
    taxi << axis.x(), axis.y(), axis.z(), 0;

    Eigen::Matrix4f N;
    N <<0, -axis.z(), axis.y(), 0,
        axis.z(), 0, -axis.x(), 0,
        -axis.y(), axis.x(), 0, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f Rodrigues;
    Rodrigues = cos(radian) * I + (1 - cos(radian)) * axi * taxi + sin(radian) * N;
     
    return Rodrigues;
}
//eye_fov         垂直可视角度
//aspect_tetio    宽高比
//zNear           近平面Z轴坐标
//zFar            远平面Z轴坐标
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    float far, near, left, right, bottom, top, fov;
    fov = eye_fov / 180 * MY_PI;    //角度转弧度
    near = -zNear; //znear是正值
    far = zFar;
    top = tan(fov / 2) * zNear;
    bottom = -top;
    right = top * aspect_ratio;
    left = -right;

    //透视->正交 perspective->orthographic
    Eigen::Matrix4f pertoorth;
    pertoorth <<    near,   0,      0,          0,
                    0,      near,   0,          0,
                    0,      0,      near + far, -near * far,
                    0,      0,      1,          0;

    //正交——移动
    Eigen::Matrix4f orth1;
    orth1 <<    1, 0, 0, -(right + left) / 2,
                0, 1, 0, -(top + bottom) / 2,
                0, 0, 1, -(near + far) / 2,
                0, 0, 0, 1;

    //正交——缩放
    Eigen::Matrix4f orth2;
    orth2 << 2 / (right - left),   0,              0,                   0,
            0,              2 / (top - bottom),    0,                   0,
            0,              0,                     2 / (near - far),    0,
            0,              0,                     0,                   1;
    projection = orth2 * orth1 * pertoorth;//注意矩阵顺序，变换从右往左依次进行

    // https ://blog.csdn.net/qq_41835314/article/details/124472121


    return projection;
}


int main(int argc, const char** argv)
{
    float angle = 0;//定义角度
    bool command_line = false;//定义命令行开关标志，默认为关掉
    std::string filename = "output.png";//定义文件名称

    Eigen::Vector3f raxis(0, 0, 1);
    double rangle = 0, ra;
    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = { 0, 0, 5 };

    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.set_rodrigues(get_rotation(raxis, rangle));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    bool rflag = false;

    std::cout << "Please enter the axis and angle:" << std::endl;
    std::cin >> raxis.x() >> raxis.y() >> raxis.z() >> ra;//定义罗德里格斯旋转轴和角

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        if (rflag) //如果按下r了，就开始绕给定任意轴旋转
            r.set_rodrigues(get_rotation(raxis, rangle));
        else
            r.set_rodrigues(get_rotation({ 0,0,1 }, 0));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'r') {
            rflag = true;
            rangle += ra;
        }
    }

    return 0;
}