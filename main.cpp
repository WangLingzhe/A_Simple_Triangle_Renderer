#include <cmath>
#include <vector>
#include<Eigen/Core>
#include<Eigen/Dense>

constexpr int width = 500;
constexpr int height = 500;

constexpr float MY_PI = 3.1415926f;

int main()
{
    std::vector<Eigen::Vector4f> model_space_positions =
    {
        {-1.0f, -1.0f, 0.0f, 1.0f},
        { 1.0f, -1.0f, 0.0f, 1.0f},
        { 0.0f,  1.0f, 0.0f, 1.0f}
    };

    Eigen::Vector3f eye(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f look_at(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f right = look_at.cross(up);

    Eigen::Matrix4f model_transformation = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f view_transformation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f view_translate = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f view_rotate = Eigen::Matrix4f::Identity();
    view_translate(0, 3) = -eye.x();
    view_translate(1, 3) = -eye.y();
    view_translate(2, 3) = -eye.z();

    view_rotate << 
        right.x(), right.y(), right.z(), 0.0f,
        up.x(), up.y(), up.z(), 0.0f,
        -look_at.x(), -look_at.y(), -look_at.z(), 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f;
    view_transformation = view_rotate * view_translate;

    float fov_degree = 45.0f;
    float fov_rad = fov_degree * MY_PI / 180.0f;
    float zNear = 0.1f;
    float zFar = 45.0f;
    float aspect = static_cast<float>(width) / static_cast<float>(height);

    Eigen::Matrix4f projection_transformation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f persp_to_ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ortho_translate = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ortho_scale = Eigen::Matrix4f::Identity();

    persp_to_ortho << 
        zNear, 0.0f, 0.0f, 0.0f,
        0.0f, zNear, 0.0f, 0.0f,
        0.0f, 0.0f, zNear + zFar, -zNear * zFar,
        0.0f, 0.0f, 1.0f, 0.0f;

    ortho_translate(2, 3) = -(zNear + zFar) / 2.0f;
    ortho_scale(0, 0) = 1.0f / (aspect * (std::tan(fov_rad) * zNear));
    ortho_scale(1, 1) = 1.0f / (std::tan(fov_rad) * zNear);
    ortho_scale(2, 2) = 2.0f / (zFar - zNear);

    projection_transformation = ortho_scale * ortho_translate * persp_to_ortho;
}
