#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>

constexpr int width = 500;
constexpr int height = 500;

constexpr float MY_PI = 3.1415926f;

static bool insideTriangle(Eigen::Vector3f p, std::vector<Eigen::Vector3f>& triangle)
{
    auto cross2D = [](Eigen::Vector2f a, Eigen::Vector2f b)
        {
            return a.x() * b.x() - a.y() * b.y();
        };

    float cross1 = cross2D(
        triangle[1] - triangle[0], triangle[1] - p.head<2>
    );

    float cross2 = cross2D(
        triangle[2] - triangle[1], triangle[2] - p.head<2>
    );

    float cross3 = cross2D(
        triangle[0] - triangle[2], triangle[0] - p.head<2>
    );

    bool all_positive = ((cross1 > 0) || (cross2 > 0) || (cross3 > 0));
    bool all_negative = ((cross1 < 0) || (cross2 < 0) || (cross3 < 0));

    return !(all_positive && all_negative);
};

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
        0.0f, 0.0f, -(zNear + zFar), -zNear * zFar,
        0.0f, 0.0f, -1.0f, 0.0f;

    ortho_translate(2, 3) = -(zNear + zFar) / 2.0f;
    ortho_scale(0, 0) = 1.0f / (aspect * (std::tan(fov_rad / 2) * zNear));
    ortho_scale(1, 1) = 1.0f / (std::tan(fov_rad / 2) * zNear);
    ortho_scale(2, 2) = 2.0f / (zFar - zNear);

    projection_transformation = ortho_scale * ortho_translate * persp_to_ortho;

    std::vector<Eigen::Vector4f> clip_space_positions;
    for (const Eigen::Vector4f& model_position : model_space_positions)
    {
        Eigen::Vector4f transformed_position = projection_transformation * view_transformation * model_transformation * model_position;

        clip_space_positions.push_back(transformed_position);
    }

    std::vector<Eigen::Vector3f> NDC_positions;
    constexpr float epsilon = 1e-6f;
    for (const Eigen::Vector4f& clip_space_position : clip_space_positions)
    {
        if (std::abs(clip_space_position.w()) > epsilon)
        {
            Eigen::Vector3f NDC_position = clip_space_position.head<3>() / clip_space_position.w();
            NDC_positions.push_back(NDC_position);
        }
    }
    
    std::vector<Eigen::Vector3f> screen_space_positions;
    for (const Eigen::Vector3f& NDC_position : NDC_positions)
    {
        Eigen::Vector3f screen_position;
        screen_position.x() = (NDC_position.x() + 1.0f) * 0.5f * static_cast<float>(width - 1);
        screen_position.y() = (1.0f - NDC_position.y()) * 0.5f * static_cast<float>(height - 1);
        screen_position.z() = (NDC_position.z() + 1.0f) * 0.5f;
        screen_space_positions.push_back(screen_position);
    }

    std::vector<Eigen::Vector3f>& triangle = screen_space_positions;

    float x_min = std::min({ triangle[0].x(), triangle[1].x(), triangle[2].x() });
    float x_max = std::max({ triangle[0].x(), triangle[1].x(), triangle[2].x() });
    float y_min = std::min({ triangle[0].y(), triangle[1].y(), triangle[2].y() });
    float y_max = std::max({ triangle[0].y(), triangle[1].y(), triangle[2].y() });

    int x_begin = std::max({ 0, static_cast<int>(std::floor(x_min)) });
    int x_end = std::min({ width - 1, static_cast<int>(std::ceil(x_max)) });
    int y_begin = std::max({ 0, static_cast<int>(std::floor(y_min)) });
    int y_end = std::min({ height - 1, static_cast<int>(std::ceil(y_max)) });


}
