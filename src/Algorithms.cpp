#include "Algorithms.h"
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/common/point_tests.h>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QTemporaryDir>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
#include <stdexcept>

namespace demo
{
    namespace
    {
        void require(bool ok, const char* message)
        {
            if (!ok)
            {
                throw std::runtime_error(message);
            }
        }

        Parameter distance(QString name, double value, double min = .0001, double max = 10000)
        {
            return {name, value, min, max, .01, 4};
        }

        Parameter count(QString name, int value, int min, int max)
        {
            return {name, double(value), double(min), double(max), 1, 0};
        }

        Normals::Ptr estimateNormals(const Cloud::ConstPtr& input, int k)
        {
            // KdTree 为每个点搜索邻域，NormalEstimation 对邻域协方差矩阵做特征值分解。
            // 最小特征值对应的特征向量就是局部法线；曲率也由特征值计算。
            pcl::NormalEstimation<Point, pcl::Normal> estimator;
            auto tree = pcl::make_shared<pcl::search::KdTree<Point>>();
            estimator.setInputCloud(input);
            estimator.setSearchMethod(tree);
            estimator.setKSearch(std::min(k, int(input->size())));
            estimator.setViewPoint(0, 0, 5); // 统一朝向观察点；不能把不同朝向的法线盲目混合。
            auto normals = pcl::make_shared<Normals>();
            estimator.compute(*normals);
            return normals;
        }

        Color heat(double value)
        {
            value = std::clamp(value, 0., 1.);
            return {static_cast<unsigned char>(40 + 215 * value),
                static_cast<unsigned char>(195 - 115 * value),
                static_cast<unsigned char>(235 - 175 * value)};
        }

        // PCL 的文件 API 使用窄字符串。先用 Qt 的 Unicode 文件 API 将文件复制到
        // 可执行文件旁的临时路径，再交给 PCL，避免直接把中文文件名传给 PCL。
        // 导出用 QSaveFile 原子提交；失败不会留下半个目标文件。
        struct IoStage
        {
            QTemporaryDir directory{QCoreApplication::applicationDirPath() + "/pcl-io-XXXXXX"};
            QString path;

            explicit IoStage(const QString& suffix)
            {
                require(directory.isValid(),
                    "Cannot create temporary IO directory beside the executable.");
                path = directory.filePath("cloud." + suffix);
                require(QFile::decodeName(QFile::encodeName(path)) == path,
                    "PCL IO requires an executable directory representable in the local code "
                    "page.");
            }

            std::string narrow() const
            {
                return QFile::encodeName(path).toStdString();
            }
        };

        void commitFile(const QString& from, const QString& to)
        {
            QFile source(from);
            QSaveFile destination(to);
            require(source.open(QIODevice::ReadOnly), "Cannot open generated temporary file.");
            require(destination.open(QIODevice::WriteOnly), "Cannot open destination for writing.");
            while (!source.atEnd())
            {
                auto data = source.read(1024 * 1024);
                require(
                    !data.isEmpty() || source.error() == QFileDevice::NoError, "File read failed.");
                require(destination.write(data) == data.size(), "File write failed.");
            }
            require(destination.commit(), "Cannot commit exported file.");
        }
    } // namespace

    const std::vector<Specification>& specifications()
    {
        static const std::vector<Specification> items = {
            {Operation::Voxel,
                "01  体素降采样",
                "pcl::VoxelGrid<PointXYZ>",
                "把点云划分为规则体素，每个体素用质心代表。通常是处理扫描数据的第一步，可降低后续算"
                "法的"
                "计"
                "算量。体素越大，保留细节越少。",
                "默认场景 → 体素大小 0.05 → 执行。观察点数下降。",
                {distance("体素边长", .05)}},
            {Operation::PassThrough,
                "02  直通滤波",
                "pcl::PassThrough<PointXYZ>",
                "按某一坐标轴保留指定区间内的点。这里沿 Z "
                "轴裁剪。可用于去掉远处背景或限定工作区域。下界必须小于上界。",
                "默认场景 → Z 下界 0.1、上界 1.5，可去除桌面。",
                {distance("Z 下界", .1, -10000), distance("Z 上界", 1.5, -10000)}},
            {Operation::Statistical,
                "03  统计离群点滤波",
                "pcl::StatisticalOutlierRemoval<PointXYZ>",
                "计算每个点到 K "
                "个近邻的平均距离，再根据全局均值与标准差剔除异常点。标准差倍数越小，剔除越严格。它"
                "假设"
                "点"
                "密度没有极端变化。",
                "默认场景 → K=20、标准差倍数 1.0。优先尝试此方法去除孤立噪声。",
                {count("近邻数 K", 20, 2, 500), distance("标准差倍数", 1, .01, 20)}},
            {Operation::RadiusOutlier,
                "04  半径离群点滤波",
                "pcl::RadiusOutlierRemoval<PointXYZ>",
                "统计每个点的指定半径内有多少邻居，邻居太少则删除。适合密度相对均匀的点云；半径过小"
                "会误"
                "删"
                "整个点云。",
                "默认场景 → 半径 0.16、最少近邻 5。",
                {distance("邻域半径", .16), count("最少近邻", 5, 1, 500)}},
            {Operation::Plane,
                "05  RANSAC 平面分割",
                "pcl::SACSegmentation + ExtractIndices",
                "随机采样拟合 "
                "ax+by+cz+d="
                "0，通过点到平面的距离判断内点。演示检测最大的一个平面，并选择保留平面或去除平面。"
                "日志"
                "输出"
                "平面系数与内点数。",
                "默认场景 → 阈值 0.025、保留平面=0。先去桌面，再执行欧式聚类。",
                {distance("距离阈值", .025),
                    count("最大迭代", 300, 10, 10000),
                    count("保留平面 0/1", 0, 0, 1)}},
            {Operation::Clusters,
                "06  欧式聚类",
                "pcl::EuclideanClusterExtraction<PointXYZ>",
                "使用 KdTree "
                "按空间距离连接点，提取连通分量。聚类容差太大会合并物体，太小会把物体拆开。未达到点"
                "数范"
                "围"
                "的分量不会出现在结果里。每个有效簇用一种辅助颜色显示。",
                "默认场景 → 去除平面 → 聚类容差 0.16、最少点数 80。预期得到三个物体。",
                {distance("聚类容差", .16),
                    count("最小簇点数", 80, 1, 500000),
                    count("最大簇点数", 100000, 1, 500000)}},
            {Operation::Normal,
                "07  法线与曲率",
                "pcl::NormalEstimation<PointXYZ, Normal>",
                "用 K "
                "邻域拟合局部平面，估计法线方向与曲率。右图以短线显示抽样法线，曲率以冷暖色显示。曲"
                "率越"
                "大"
                "通常表示邻域越不平整。导出点云只保存 XYZ。",
                "切换为球面示例 → K=20、显示长度 0.12。拖动视角观察法线。",
                {count("近邻数 K", 20, 3, 500), distance("显示长度", .12)}},
            {Operation::Fpfh,
                "08  FPFH 特征",
                "pcl::FPFHEstimation<PointXYZ, Normal, FPFHSignature33>",
                "先计算法线，再根据邻域法线间的角度关系生成每点 33 "
                "维直方图。特征用于点云匹配与识别。本例展示计算方式、有效特征数及第一维特征的颜色映"
                "射；"
                "它"
                "不是类别识别器。",
                "球面示例 → 法线 K=20、特征半径 0.25。特征半径应大于法线估计的典型邻域。",
                {count("法线近邻 K", 20, 3, 500), distance("特征半径", .25)}},
            {Operation::Mls,
                "09  MLS 曲面平滑",
                "pcl::MovingLeastSquares<PointXYZ, PointNormal>",
                "在搜索半径内拟合局部多项式，把点投影到平滑曲面并计算法线。半径过小会缺少支持点，过"
                "大会"
                "抹"
                "平细节。本例不启用额外上采样。",
                "球面示例 → 搜索半径 0.18、多项式阶数 2。",
                {distance("搜索半径", .18), count("多项式阶数", 2, 1, 3)}},
            {Operation::Triangulate,
                "10  贪婪三角化",
                "pcl::GreedyProjectionTriangulation<PointNormal>",
                "先估计法线，再把邻域投影到局部切平面生成三角网格。只适合表面采样，不能保证水密，也"
                "不会"
                "自"
                "动跨越大孔洞。网格可单独导出为 PLY。",
                "球面示例 → 搜索半径 0.22、mu=2.5、法线 K=20。",
                {distance("搜索半径", .22),
                    distance("mu 邻域倍数", 2.5, 1, 20),
                    count("法线近邻 K", 20, 3, 500)}},
            {Operation::KNearest,
                "11  KdTree K 近邻",
                "pcl::KdTreeFLANN<PointXYZ>::nearestKSearch",
                "以点云中的某个点作为查询点，检索最近的 K 个点。黄色是近邻，红色是查询点。PCL "
                "返回的距离是平方距离；需要真实距离时再开平方。",
                "输入查询点索引和 K。索引从 0 开始，超出当前点数会报错。",
                {count("查询点索引", 0, 0, 499999), count("近邻数 K", 40, 1, 5000)}},
            {Operation::RadiusSearch,
                "12  KdTree 半径搜索",
                "pcl::KdTreeFLANN<PointXYZ>::radiusSearch",
                "返回查询点指定半径内的所有点，用于局部几何分析和交互选择。查询点显示红色，搜索到的"
                "点显"
                "示"
                "黄色。",
                "球面示例 → 点索引 0、搜索半径 0.3。",
                {count("查询点索引", 0, 0, 499999), distance("搜索半径", .3)}},
            {Operation::Octree,
                "13  Octree 占据体素",
                "pcl::octree::OctreePointCloudSearch<PointXYZ>",
                "构建八叉树空间索引，提取有点占据的体素中心。它演示空间划分：结果是体素中心，并非 "
                "VoxelGrid 的点簇质心。日志同时展示第一个点所在体素的命中数。",
                "默认场景 → 分辨率 0.12，与体素降采样的结果作比较。",
                {distance("体素分辨率", .12)}},
            {Operation::Transform,
                "14  刚体坐标变换",
                "pcl::transformPointCloud + Eigen::Matrix4f",
                "构造 4×4 齐次矩阵，将点云绕 Z 轴旋转后平移。矩阵左上 3×3 "
                "为旋转，最后一列前三项为平移。旋转角在界面使用度，代码会转成弧度。",
                "设 X 平移 0.4、Z 轴旋转 20°，与左侧原始点云对照。",
                {distance("X 平移", .4, -10000),
                    distance("Y 平移", 0, -10000),
                    distance("Z 平移", 0, -10000),
                    {"绕 Z 旋转 / 度", 20, -180, 180, 1, 1}}},
            {Operation::Icp,
                "15  ICP 精配准",
                "pcl::IterativeClosestPoint<PointXYZ, PointXYZ>",
                "反复寻找最近点对应并求解刚体变换，最小化点对距离。需有较好的初始位置；hasConverged"
                " "
                "只表示满足停止条件，不代表配准一定正确。日志输出收敛状态、fitness "
                "和变换矩阵，绿色为目标。",
                "点击“ICP 示例”准备错位源点云与目标 → 选 ICP → 执行。也可单独加载目标 PCD / PLY。",
                {count("最大迭代", 80, 1, 1000),
                    distance("最大对应距离", .3),
                    {"变换收敛阈值", .00000001, .000000001, .1, .00000001, 9},
                    {"均方误差阈值", .000001, .000000001, .1, .000001, 9}}}};
        return items;
    }

    Cloud::Ptr makeScene()
    {
        auto cloud = pcl::make_shared<Cloud>();
        std::mt19937 rng(42); // 固定种子，便于不同算法使用相同输入、复现结果。
        std::normal_distribution<float> noise(0, .005f);
        for (int x = -24; x <= 24; ++x)
        {
            for (int y = -18; y <= 18; ++y)
            {
                cloud->push_back(Point(x * .065f, y * .065f, noise(rng)));
            }
        }
        const Eigen::Vector3f centers[] = {
            {-.8f, -.35f, .55f}, {.0f, .45f, .72f}, {.85f, -.25f, .5f}};
        std::uniform_real_distribution<float> unit(0.f, 1.f);
        for (int object = 0; object < 3; ++object)
        {
            for (int i = 0; i < 720; ++i)
            {
                float z = 2.f * unit(rng) - 1.f;
                float angle = 6.2831853f * unit(rng);
                float radius = .22f + noise(rng);
                float xy = std::sqrt(1.f - z * z);
                const auto& c = centers[object];
                cloud->push_back(Point(c.x() + radius * xy * std::cos(angle),
                    c.y() + radius * xy * std::sin(angle),
                    c.z() + radius * z));
            }
        }
        std::uniform_real_distribution<float> scatter(-2.4f, 2.4f);
        for (int i = 0; i < 65; ++i)
        {
            cloud->push_back(Point(scatter(rng), scatter(rng), .5f + unit(rng) * 1.8f));
        }
        return cloud;
    }

    Cloud::Ptr makeSphere()
    {
        auto cloud = pcl::make_shared<Cloud>();
        std::mt19937 rng(17);
        std::normal_distribution<float> noise(0, .007f);
        // Fibonacci sphere 比经纬网在极点的密度更均匀，适合展示邻域算法。
        constexpr int n = 3600;
        for (int i = 0; i < n; ++i)
        {
            float z = 1.f - 2.f * (i + .5f) / n;
            float a = 2.39996323f * i;
            float xy = std::sqrt(1.f - z * z);
            float r = 1.f + noise(rng);
            cloud->push_back(Point(r * xy * std::cos(a), r * xy * std::sin(a), r * z));
        }
        return cloud;
    }

    Cloud::Ptr makeRegistrationTarget()
    {
        auto cloud = makeSphere();
        for (auto& p : *cloud)
        {
            // 非对称曲面减少球体旋转对称性导致的 ICP 歧义。
            p.x *= 1.1f + .2f * p.z;
            p.y *= .65f;
            p.z += .12f * p.x * p.x;
        }
        return cloud;
    }

    Cloud::Ptr displaced(const Cloud::ConstPtr& input)
    {
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        m.block<3, 3>(0, 0) = Eigen::AngleAxisf(.10f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        m(0, 3) = .09f;
        m(1, 3) = -.06f;
        m(2, 3) = .045f;
        auto result = pcl::make_shared<Cloud>();
        pcl::transformPointCloud(*input, *result, m);
        return result;
    }

    Cloud::Ptr readCloud(const QString& filename)
    {
        QString suffix = QFileInfo(filename).suffix().toLower();
        require(suffix == "pcd" || suffix == "ply", "Only PCD and PLY files are supported.");
        IoStage stage(suffix);
        require(QFile::copy(filename, stage.path), "Cannot read or stage input file.");
        pcl::PCLPointCloud2 blob;
        int rc = suffix == "pcd" ? pcl::io::loadPCDFile(stage.narrow(), blob)
                                 : pcl::io::loadPLYFile(stage.narrow(), blob);
        require(rc >= 0, "PCL could not parse this point cloud.");
        for (const auto* field : {"x", "y", "z"})
        {
            require(pcl::getFieldIndex(blob, field) >= 0, "Input must have x, y and z fields.");
        }
        require(std::uint64_t(blob.width) * blob.height <= 500000,
            "This teaching demo supports up to 500,000 points. Downsample larger input first.");
        auto raw = pcl::make_shared<Cloud>();
        pcl::fromPCLPointCloud2(blob, *raw);
        auto clean = pcl::make_shared<Cloud>();
        std::vector<int> indices;
        raw->is_dense = false; // 文件的 is_dense 标记不一定可靠，强制检查有限坐标。
        pcl::removeNaNFromPointCloud(*raw, *clean, indices);
        require(!clean->empty(), "Input has no finite XYZ points.");
        return clean;
    }

    void writeCloud(const QString& filename, const Cloud::ConstPtr& cloud)
    {
        require(cloud && !cloud->empty(), "There is no point cloud to export.");
        QString suffix = QFileInfo(filename).suffix().toLower();
        require(suffix == "pcd" || suffix == "ply", "Choose a .pcd or .ply extension.");
        IoStage stage(suffix);
        int rc = suffix == "pcd" ? pcl::io::savePCDFileBinary(stage.narrow(), *cloud)
                                 : pcl::io::savePLYFileBinary(stage.narrow(), *cloud);
        require(rc >= 0, "PCL failed to write the point cloud.");
        commitFile(stage.path, filename);
    }

    void writeMesh(const QString& filename, const pcl::PolygonMesh& mesh)
    {
        require(!mesh.polygons.empty(), "There is no mesh to export.");
        require(QFileInfo(filename).suffix().compare("ply", Qt::CaseInsensitive) == 0,
            "Mesh output must use .ply.");
        IoStage stage("ply");
        require(
            pcl::io::savePLYFileBinary(stage.narrow(), mesh) >= 0, "PCL failed to write the mesh.");
        commitFile(stage.path, filename);
    }

    Result execute(
        Operation op, const Cloud::ConstPtr& input, const Values& p, const Cloud::ConstPtr& target)
    {
        require(input && !input->empty(),
            "Input cloud is empty. Load a file or generate a sample first.");
        // 界面已经限制参数范围；算法层仍检查一次，避免其他调用方传入 NaN 或非法计数。
        for (const auto& spec : specifications())
        {
            if (spec.operation == op)
            {
                for (std::size_t i = 0; i < spec.parameters.size(); ++i)
                {
                    const auto& limit = spec.parameters[i];
                    require(std::isfinite(p[i]) && p[i] >= limit.minimum && p[i] <= limit.maximum,
                        "A parameter is outside its supported range.");
                    require(limit.decimals != 0 || std::floor(p[i]) == p[i],
                        "Count and index parameters must be integers.");
                }
            }
        }
        if (op == Operation::Mls || op == Operation::Fpfh || op == Operation::Triangulate)
        {
            require(input->size() <= 100000,
                "Downsample to at most 100,000 points before this expensive operation.");
        }
        Result out;
        // 每个分支都创建自己的算法对象，setInputCloud 只读取输入，不会原地修改它。
        switch (op)
        {
            case Operation::Voxel:
            {
                // 三个方向的 leaf size 都是坐标单位，不能为零。输出是体素内点的质心，
                // 与 Octree 示例输出的规则体素几何中心不同；无序点云的输出顺序也会改变。
                require(p[0] > 0, "Voxel size must be positive.");
                pcl::VoxelGrid<Point> filter;
                filter.setInputCloud(input);
                filter.setLeafSize(float(p[0]), float(p[0]), float(p[0]));
                filter.filter(*out.cloud); // 每个非空体素输出一个质心。
                break;
            }
            case Operation::PassThrough:
            {
                // 字段名必须与 PointXYZ 的坐标字段一致；若要沿 X 或 Y 裁剪，改为 "x" / "y"。
                // 默认输出区间内的点；setNegative(true) 可改为输出区间外的点。
                require(p[0] < p[1], "Z minimum must be smaller than Z maximum.");
                pcl::PassThrough<Point> filter;
                filter.setInputCloud(input);
                filter.setFilterFieldName("z");
                filter.setFilterLimits(float(p[0]), float(p[1]));
                filter.filter(*out.cloud);
                break;
            }
            case Operation::Statistical:
            {
                // MeanK 决定局部统计范围。StddevMulThresh 越小，距离均值偏离全局分布的
                // 点越容易被剔除；不是按坐标阈值逐点裁剪，也不需要预先知道噪声位置。
                require(input->size() >= 4, "Statistical filtering requires at least 4 points.");
                pcl::StatisticalOutlierRemoval<Point> filter;
                filter.setInputCloud(input);
                filter.setMeanK(std::min(int(p[0]), int(input->size()) - 1));
                filter.setStddevMulThresh(p[1]);
                filter.filter(*out.cloud);
                break;
            }
            case Operation::RadiusOutlier:
            {
                // 与统计滤波的区别：这里直接使用固定空间尺度，适合已知点密度的数据。
                // 若扫描距离变化造成点密度差异很大，同一组半径与最少近邻数未必适合全场景。
                pcl::RadiusOutlierRemoval<Point> filter;
                filter.setInputCloud(input);
                filter.setRadiusSearch(p[0]);
                filter.setMinNeighborsInRadius(int(p[1]));
                filter.filter(*out.cloud);
                break;
            }
            case Operation::Plane:
            {
                // SACMODEL_PLANE 指定平面几何模型，SAC_RANSAC 指定随机采样求解策略。
                // indices 是原输入点的索引，ExtractIndices 使用这些索引提取平面或其补集。
                require(input->size() >= 3, "Plane segmentation requires at least 3 points.");
                auto indices = pcl::make_shared<pcl::PointIndices>();
                auto coefficients = pcl::make_shared<pcl::ModelCoefficients>();
                pcl::SACSegmentation<Point> segmentation;
                segmentation.setOptimizeCoefficients(true);
                segmentation.setModelType(pcl::SACMODEL_PLANE);
                segmentation.setMethodType(pcl::SAC_RANSAC);
                segmentation.setDistanceThreshold(p[0]);
                segmentation.setMaxIterations(int(p[1]));
                segmentation.setInputCloud(input);
                segmentation.segment(*indices, *coefficients);
                require(!indices->indices.empty(),
                    "RANSAC did not find a plane. Try increasing the distance threshold.");
                pcl::ExtractIndices<Point> extract;
                extract.setInputCloud(input);
                extract.setIndices(indices);
                extract.setNegative(p[2] < .5); // true=输出补集，去掉桌面；false=仅保留平面。
                extract.filter(*out.cloud);
                QStringList values;
                for (float c : coefficients->values)
                {
                    values << QString::number(c, 'g', 6);
                }
                out.detail = QString("平面内点 %1 / %2；[a,b,c,d] = [%3]")
                                 .arg(indices->indices.size())
                                 .arg(input->size())
                                 .arg(values.join(", "));
                break;
            }
            case Operation::Clusters:
            {
                // 聚类需要空间邻域查询，因此显式创建搜索树。输出是“索引集合的集合”，
                // 而不是直接给每个点写标签。下面逐簇复制点，并同步维护可视化颜色数组。
                require(p[1] <= p[2], "Minimum cluster size must not exceed maximum cluster size.");
                auto tree = pcl::make_shared<pcl::search::KdTree<Point>>();
                tree->setInputCloud(input);
                pcl::EuclideanClusterExtraction<Point> extraction;
                extraction.setInputCloud(input);
                extraction.setSearchMethod(tree);
                extraction.setClusterTolerance(p[0]);
                extraction.setMinClusterSize(int(p[1]));
                extraction.setMaxClusterSize(int(p[2]));
                std::vector<pcl::PointIndices> clusters;
                extraction.extract(clusters);
                const Color palette[] = {{55, 199, 230},
                    {255, 184, 75},
                    {150, 128, 255},
                    {75, 215, 147},
                    {244, 113, 150},
                    {230, 220, 100}};
                QStringList sizes;
                for (std::size_t c = 0; c < clusters.size(); ++c)
                {
                    sizes << QString::number(clusters[c].indices.size());
                    for (int index : clusters[c].indices)
                    {
                        out.cloud->push_back((*input)[index]);
                        out.colors.push_back(palette[c % 6]);
                    }
                }
                out.detail = QString("有效簇 %1；各簇点数 [%2]；未满足点数条件的点已排除。")
                                 .arg(clusters.size())
                                 .arg(sizes.join(", "));
                break;
            }
            case Operation::Normal:
            {
                // K 过小会对噪声敏感，过大会把棱边两侧混为一个局部平面。
                // 输入和法线必须一一对应；显示时抽样短线，计算与结果数组不做抽样。
                require(input->size() >= 4, "Normal estimation requires at least 4 points.");
                *out.cloud = *input;
                out.normals = estimateNormals(input, int(p[0]));
                out.normalScale = p[1];
                int valid = 0;
                for (const auto& n : *out.normals)
                {
                    bool finite = std::isfinite(n.normal_x) && std::isfinite(n.normal_y) &&
                        std::isfinite(n.normal_z);
                    valid += finite;
                    out.colors.push_back(finite ? heat(n.curvature * 15.) : Color{110, 110, 110});
                }
                out.detail = QString("有效法线 %1 / %2；暖色表示较大的局部曲率。")
                                 .arg(valid)
                                 .arg(input->size());
                break;
            }
            case Operation::Fpfh:
            {
                // FPFH 的输入包括点坐标与同序法线。搜索半径控制特征描述的空间尺度，
                // 通常应大于法线邻域尺度；过小的邻域可能产生无效特征，不能直接用于匹配。
                require(input->size() >= 4, "FPFH estimation requires at least 4 points.");
                *out.cloud = *input;
                auto normals = estimateNormals(input, int(p[0]));
                pcl::FPFHEstimation<Point, pcl::Normal, pcl::FPFHSignature33> estimator;
                estimator.setInputCloud(input);
                estimator.setInputNormals(normals);
                estimator.setSearchMethod(pcl::make_shared<pcl::search::KdTree<Point>>());
                estimator.setRadiusSearch(p[1]);
                pcl::PointCloud<pcl::FPFHSignature33> features;
                estimator.compute(features);
                int valid = 0;
                for (const auto& f : features)
                {
                    bool finite = std::all_of(std::begin(f.histogram),
                        std::end(f.histogram),
                        [](float v)
                        {
                            return std::isfinite(v);
                        });
                    valid += finite;
                    out.colors.push_back(
                        finite ? heat(f.histogram[0] / 100.) : Color{110, 110, 110});
                }
                out.detail = QString(
                    "FPFH 特征维数=33；有效特征 %1 / %2。颜色映射直方图第一维，不表示类别。")
                                 .arg(valid)
                                 .arg(features.size());
                break;
            }
            case Operation::Mls:
            {
                // MLS 的输出类型使用 PointNormal，因此能同时读取平滑坐标、法线和曲率。
                // 本例保留原采样附近的点，不配置上采样；输出点数不承诺与输入完全一致。
                require(input->size() >= 6, "MLS requires at least 6 points.");
                pcl::MovingLeastSquares<Point, pcl::PointNormal> mls;
                mls.setInputCloud(input);
                mls.setSearchMethod(pcl::make_shared<pcl::search::KdTree<Point>>());
                mls.setSearchRadius(p[0]);
                mls.setPolynomialOrder(int(p[1]));
                mls.setComputeNormals(true);
                pcl::PointCloud<pcl::PointNormal> smooth;
                mls.process(smooth);
                out.normals = pcl::make_shared<Normals>();
                // 某些稀疏邻域无法拟合，必须显式过滤无效的输出点与法线。
                for (const auto& v : smooth)
                {
                    if (!pcl::isFinite(v))
                    {
                        continue;
                    }
                    out.cloud->push_back(Point(v.x, v.y, v.z));
                    pcl::Normal normal;
                    normal.normal_x = v.normal_x;
                    normal.normal_y = v.normal_y;
                    normal.normal_z = v.normal_z;
                    normal.curvature = v.curvature;
                    out.normals->push_back(normal);
                }
                out.detail = "MLS 已输出平滑 XYZ 与法线；稀疏区域可能被跳过。";
                break;
            }
            case Operation::Triangulate:
            {
                // GP3 输入必须同时具有坐标与法线。concatenateFields 按相同点索引拼接字段，
                // 不能把经过不同筛选、顺序不同的点云和法线拼接。搜索半径控制可连接边的尺度；
                // mu 根据局部近邻距离调整搜索范围；角度限制用于减少过尖或跨越折角的三角形。
                require(input->size() >= 6, "Triangulation requires at least 6 points.");
                *out.cloud = *input;
                auto normals = estimateNormals(input, int(p[2]));
                auto points = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
                pcl::concatenateFields(*input, *normals, *points);
                auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
                tree->setInputCloud(points);
                pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
                gp3.setInputCloud(points);
                gp3.setSearchMethod(tree);
                gp3.setSearchRadius(p[0]);
                gp3.setMu(p[1]);
                gp3.setMaximumNearestNeighbors(100);
                gp3.setMaximumSurfaceAngle(float(3.141592653589793 / 4));
                gp3.setMinimumAngle(float(3.141592653589793 / 18));
                gp3.setMaximumAngle(float(2 * 3.141592653589793 / 3));
                gp3.setNormalConsistency(false); // 不假定输入法线已全局定向。
                out.mesh = pcl::make_shared<pcl::PolygonMesh>();
                gp3.reconstruct(*out.mesh);
                out.detail = QString("生成 %1 个三角面；可使用“导出网格”保存 PLY。")
                                 .arg(out.mesh->polygons.size());
                break;
            }
            case Operation::KNearest:
            case Operation::RadiusSearch:
            {
                // 返回 indices 可直接索引原始点云，squaredDistances 中的每个元素与之同序。
                // nearestKSearch 的返回值可能小于请求 K；半径搜索的结果数由局部密度决定。
                require(
                    p[0] >= 0 && p[0] < input->size(), "Query index is outside the current cloud.");
                *out.cloud = *input;
                pcl::KdTreeFLANN<Point> tree;
                tree.setInputCloud(input);
                pcl::Indices indices;
                std::vector<float> squaredDistances;
                const auto& query = (*input)[int(p[0])];
                int found = op == Operation::KNearest
                    ? tree.nearestKSearch(
                          query, std::min(int(p[1]), int(input->size())), indices, squaredDistances)
                    : tree.radiusSearch(query, p[1], indices, squaredDistances);
                out.colors.assign(input->size(), {64, 95, 123});
                for (int index : indices)
                {
                    out.colors[index] = {255, 202, 70};
                }
                out.colors[int(p[0])] = {255, 77, 100};
                out.detail = QString("搜索命中 %1 点；查询点 (%2, %3, %4)。PCL 返回的是平方距离。")
                                 .arg(found)
                                 .arg(query.x)
                                 .arg(query.y)
                                 .arg(query.z);
                break;
            }
            case Operation::Octree:
            {
                // resolution 决定叶节点尺寸。addPointsFromInputCloud 构建索引；改变输入之后
                // 需要重建索引。该例只显示占据体素中心，没有把原始点颜色或其他属性聚合进去。
                pcl::octree::OctreePointCloudSearch<Point> octree(p[0]);
                octree.setInputCloud(input);
                octree.addPointsFromInputCloud();
                Cloud::VectorType centers;
                octree.getOccupiedVoxelCenters(centers);
                for (const auto& v : centers)
                {
                    out.cloud->push_back(v);
                }
                pcl::Indices indices;
                octree.voxelSearch((*input)[0], indices);
                out.detail =
                    QString("占据体素 %1 个；第一个输入点所在体素包含 %2 点。结果显示体素中心。")
                        .arg(centers.size())
                        .arg(indices.size());
                break;
            }
            case Operation::Transform:
            {
                // 列向量约定：R 先作用于输入，t 再加到结果上，因此 T=[R t; 0 1]。
                // Eigen 的 AngleAxisf 接受弧度；UI 使用角度，必须显式进行单位换算。
                Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
                matrix.block<3, 3>(0, 0) = Eigen::AngleAxisf(
                    float(p[3] * 3.141592653589793 / 180.), Eigen::Vector3f::UnitZ())
                                               .toRotationMatrix();
                matrix(0, 3) = float(p[0]);
                matrix(1, 3) = float(p[1]);
                matrix(2, 3) = float(p[2]);
                pcl::transformPointCloud(*input, *out.cloud, matrix);
                out.transform = matrix;
                break;
            }
            case Operation::Icp:
            {
                // 源云是待移动对象，目标云固定。align 的输出为变换后的源云，不是两云合并。
                // 最大对应距离太小会找不到匹配，太大会引入错误对应；ICP 不提供全局初始定位。
                // 使用不同单位的数据时，要同时调整对应距离与收敛尺度，先做粗配准更可靠。
                require(target && target->size() >= 4 && input->size() >= 4,
                    "ICP requires source and target clouds with at least 4 points. Prepare the ICP "
                    "sample or load a target.");
                pcl::IterativeClosestPoint<Point, Point> icp;
                icp.setInputSource(input);
                icp.setInputTarget(target);
                icp.setMaximumIterations(int(p[0]));
                icp.setMaxCorrespondenceDistance(p[1]);
                icp.setTransformationEpsilon(p[2]);
                icp.setEuclideanFitnessEpsilon(p[3]);
                icp.align(*out.cloud);
                out.target = target;
                out.converged = icp.hasConverged();
                out.fitness = icp.getFitnessScore(p[1] * p[1]);
                out.transform = icp.getFinalTransformation();
                out.detail = QString(
                    "ICP 收敛=%1；fitness=%2（最大对应距离内的均方距离）。收敛不等于全局最优。")
                                 .arg(out.converged ? "是" : "否")
                                 .arg(out.fitness, 0, 'g', 9);
                break;
            }
        }
        if (op == Operation::Transform || op == Operation::Icp)
        {
            std::ostringstream stream;
            stream << out.transform;
            out.detail +=
                "\n变换矩阵（输出 = T × 输入）：\n" + QString::fromStdString(stream.str());
        }
        if (out.cloud->empty())
        {
            out.detail += "\n结果为空：当前参数删除了全部点。可以撤销或恢复原始点云后调整参数。";
        }
        return out;
    }
} // namespace demo
