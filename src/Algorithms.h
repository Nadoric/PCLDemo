#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Core>
#include <QString>
#include <array>
#include <vector>

namespace demo
{
    // 教学项目统一采用 XYZ，所有距离参数与输入坐标使用相同单位。
    // 文件中的 RGB、强度、时间戳等其他字段不会参与本项目的处理与导出。
    using Point = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<Point>;
    using Normals = pcl::PointCloud<pcl::Normal>;

    struct Color
    {
        unsigned char r = 57;
        unsigned char g = 189;
        unsigned char b = 227;
    };

    enum class Operation
    {
        Voxel,
        PassThrough,
        Statistical,
        RadiusOutlier,
        Plane,
        Clusters,
        Normal,
        Fpfh,
        Mls,
        Triangulate,
        KNearest,
        RadiusSearch,
        Octree,
        Transform,
        Icp
    };

    struct Parameter
    {
        QString name;
        double value = 0;
        double minimum = 0;
        double maximum = 100;
        double step = .01;
        int decimals = 3;
    };

    struct Specification
    {
        Operation operation;
        QString title;
        QString api;
        QString description;
        QString recipe;
        std::vector<Parameter> parameters;
    };

    const std::vector<Specification>& specifications();

    struct Result
    {
        Cloud::Ptr cloud{new Cloud};
        Normals::Ptr normals;
        pcl::PolygonMesh::Ptr mesh;
        Cloud::ConstPtr target;    // ICP 的目标以绿色叠加显示，不写进输出点云。
        std::vector<Color> colors; // 可视化辅助色；不是原文件的颜色字段。
        QString detail;
        double normalScale = .1;
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        bool converged = false;
        double fitness = 0;
    };

    using Values = std::array<double, 4>;

    Cloud::Ptr makeScene();              // 平面 + 三个目标 + 少量离群点
    Cloud::Ptr makeSphere();             // 连续曲面，适合 MLS / 法线 / 三角化
    Cloud::Ptr makeRegistrationTarget(); // 非对称几何体，适合 ICP
    Cloud::Ptr displaced(const Cloud::ConstPtr& input);
    Cloud::Ptr readCloud(const QString& filename);
    void writeCloud(const QString& filename, const Cloud::ConstPtr& cloud);
    void writeMesh(const QString& filename, const pcl::PolygonMesh& mesh);

    // 算法与界面分离：所有 PCL 对象都在调用线程内创建，输入只读；
    // UI 可以安全地将 execute 放进 QtConcurrent::run，不跨线程修改点云。
    Result execute(Operation op,
        const Cloud::ConstPtr& input,
        const Values& p,
        const Cloud::ConstPtr& target = {});
} // namespace demo
