#include "RgbdAlgorithms.h"
#include <opencv2/core/ocl.hpp>

#include <opencv2/features.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <pcl/common/point_tests.h>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo
{
    namespace
    {
        void cvRequire(bool condition, const char* message)
        {
            if (!condition)
            {
                throw std::runtime_error(message);
            }
        }

        void validateFrame(const RgbdFrame& frame)
        {
            cvRequire(!frame.bgr.empty() && frame.bgr.type() == CV_8UC3,
                "Color image must be an 8-bit BGR image.");
            cvRequire(frame.depth.type() == CV_32FC1 && frame.depth.size() == frame.bgr.size(),
                "Depth must be float meters and have the same size as the registered color image.");
            cvRequire(std::isfinite(frame.camera.fx) && frame.camera.fx > 0.0 &&
                    std::isfinite(frame.camera.fy) && frame.camera.fy > 0.0 &&
                    std::isfinite(frame.camera.cx) && std::isfinite(frame.camera.cy),
                "Camera focal lengths must be positive and all intrinsics must be finite.");
        }

        cv::Mat decodeImage(const QString& filename, int mode)
        {
            // 使用 QFile 读取 Unicode 路径，再用 imdecode 解码，绕开 Windows 窄字符路径问题。
            QFile file(filename);
            cvRequire(file.open(QIODevice::ReadOnly), "Cannot open image file.");
            const QByteArray bytes = file.readAll();
            cvRequire(!bytes.isEmpty(), "Image file is empty.");
            const std::vector<unsigned char> buffer(bytes.begin(), bytes.end());
            cv::Mat image = cv::imdecode(buffer, mode);
            cvRequire(!image.empty(), "OpenCV cannot decode this image.");
            cvRequire(image.total() <= 4194304, "Use images with at most 4,194,304 pixels.");
            return image;
        }

        cv::Mat foregroundMask(const RgbdFrame& frame, double nearDepth, double farDepth)
        {
            cvRequire(nearDepth > 0.0 && nearDepth < farDepth,
                "Foreground minimum depth must be positive and smaller than maximum depth.");
            cv::Mat mask;
            cv::inRange(frame.depth, nearDepth, farDepth, mask);
            return mask;
        }

        void validateParameters(CvOperation operation, const Values& values)
        {
            for (const auto& specification : cvSpecifications())
            {
                if (specification.operation != operation)
                {
                    continue;
                }

                for (std::size_t i = 0; i < specification.parameters.size(); ++i)
                {
                    const auto& parameter = specification.parameters[i];
                    cvRequire(std::isfinite(values[i]) && values[i] >= parameter.minimum &&
                            values[i] <= parameter.maximum,
                        "An OpenCV parameter is outside its supported range.");
                    cvRequire(parameter.decimals != 0 || std::floor(values[i]) == values[i],
                        "Kernel sizes and counts must be integers.");
                }
                return;
            }

            throw std::runtime_error("Unknown OpenCV operation.");
        }
    } // namespace

    const std::vector<CvSpecification>& cvSpecifications()
    {
        static const std::vector<CvSpecification> specifications = {
            {CvOperation::BackProject,
                "01  RGB-D → 彩色点云",
                "cv::Mat + 针孔模型 → pcl::PointCloud",
                "用相机内参将像素 (u,v) 和深度 Z 反投影为三维点：X=(u-cx)Z/fx，"
                "Y=(v-cy)Z/fy。坐标采用相机系：X 向右，Y 向下，Z 向前。"
                "RGB 与深度必须已配准。本页保留像素颜色，可发送到 PCL 继续处理。",
                {}},
            {CvOperation::MedianDepth,
                "02  深度中值滤波 → 点云",
                "cv::medianBlur",
                "去除深度图中的脉冲噪声，再生成点云。浮点输入只允许 3×3 或 5×5 核。"
                "原有无效深度仍保持为 0；本例不把滤波当成可靠的孔洞修复。",
                {{"核大小 3 / 5", 5, 3, 5, 2, 0}}},
            {CvOperation::BilateralDepth,
                "03  双边保边平滑 → 点云",
                "cv::inpaint + cv::bilateralFilter",
                "同时考虑像素距离和深度差异，在平滑表面噪声时尽量保留深度跳变。"
                "先暂时修补无效邻域以避免零深度污染，滤波后恢复原始无效掩膜。"
                "深度 sigma 的单位是米，不是毫米。",
                {{"邻域直径 / 奇数", 5, 3, 15, 2, 0},
                    {"深度 sigma / 米", .04, .001, 1, .005, 3},
                    {"空间 sigma / 像素", 3, .5, 30, .5, 1}}},
            {CvOperation::DepthRange,
                "04  深度阈值分割 → 点云",
                "cv::inRange + 像素掩膜反投影",
                "用深度区间建立前景掩膜，只把前景像素转换为点云。"
                "这是二维图像选择三维空间区域的直接示例。默认示例中，背景深度约为 2.8 米。",
                {{"前景最小深度 / 米", .2, .001, 50, .1, 3},
                    {"前景最大深度 / 米", 2.3, .002, 60, .1, 3}}},
            {CvOperation::Morphology,
                "05  形态学清理 → 点云",
                "cv::morphologyEx / MORPH_OPEN / MORPH_CLOSE",
                "先按深度生成掩膜，开运算删除小块噪声，闭运算连接小间隙。"
                "形态学作用于掩膜，不会凭空补出三维深度；没有有效深度的像素仍不会生成点。",
                {{"前景最小深度 / 米", .2, .001, 50, .1, 3},
                    {"前景最大深度 / 米", 2.3, .002, 60, .1, 3},
                    {"形态学核 / 奇数", 5, 3, 31, 2, 0}}},
            {CvOperation::Canny,
                "06  Canny 边缘 → 三维边缘",
                "cv::cvtColor + GaussianBlur + Canny",
                "将 RGB 转灰度并高斯平滑，执行 Canny，随后用同一像素的深度提取三维边缘点。"
                "这里检测的是颜色边缘；颜色纹理边缘不一定对应物体几何边界。",
                {{"低阈值", 40, 0, 254, 1, 0}, {"高阈值", 100, 1, 255, 1, 0}}},
            {CvOperation::HsvMask,
                "07  HSV 颜色分割 → 点云",
                "cv::cvtColor / COLOR_BGR2HSV + inRange",
                "按色相与最低饱和度选择像素，再提取对应三维点。OpenCV 的 8 位色相范围为 0~179。"
                "下界大于上界时跨越色相零点，可用 170~10 提取红色。低亮度像素也会被排除。",
                {{"色相下界", 170, 0, 179, 1, 0},
                    {"色相上界", 10, 0, 179, 1, 0},
                    {"最小饱和度", 100, 0, 255, 1, 0},
                    {"最小亮度", 50, 0, 255, 1, 0}}},
            {CvOperation::Contours,
                "08  轮廓 / 包围框 → 三维尺寸",
                "cv::findContours + boundingRect → PCL XYZ 范围",
                "按深度提取前景并做开运算，筛选二维外轮廓。在图像上标出包围框，同时提取各目标点云，"
                "在日志中列出各目标在相机坐标轴方向的 XYZ 尺寸。不是旋转不变的真实物体长宽高。",
                {{"前景最小深度 / 米", .2, .001, 50, .1, 3},
                    {"前景最大深度 / 米", 2.3, .002, 60, .1, 3},
                    {"最小轮廓面积 / 像素", 100, 1, 1000000, 10, 0}}},
            {CvOperation::Orb,
                "09  ORB 特征 → 三维特征点",
                "cv::ORB::detectAndCompute + 深度关联",
                "计算二维 ORB 关键点和 32 "
                "字节二进制描述子，再把具有有效深度的关键点像素提升到三维。"
                "这里只演示检测和 2D/3D 关联，不执行描述子匹配、位姿求解或配准。",
                {{"最大关键点数", 400, 20, 3000, 20, 0}, {"FAST 阈值", 12, 1, 100, 1, 0}}},
            {CvOperation::ProjectCloud,
                "10  PCL 点云 → 深度图",
                "针孔投影 + Z-buffer + cv::applyColorMap",
                "读取 PCL "
                "工作区当前点云，用本页相机内参回投影到图像。多个点落到同一个像素时保留最近深度。"
                "不会对任意世界坐标点云自动估计相机姿态。建议先将 RGB-D 点云送到 "
                "PCL，降采样后再回投影。",
                {}}};
        return specifications;
    }

    Values cvDefaults(CvOperation operation)
    {
        Values values{};
        for (const auto& specification : cvSpecifications())
        {
            if (specification.operation == operation)
            {
                for (std::size_t i = 0; i < specification.parameters.size(); ++i)
                {
                    values[i] = specification.parameters[i].value;
                }
            }
        }
        return values;
    }

    RgbdFrame makeRgbdExample()
    {
        RgbdFrame frame;
        frame.bgr = cv::Mat(240, 320, CV_8UC3, cv::Scalar(58, 63, 72));
        frame.depth = cv::Mat(240, 320, CV_32FC1, cv::Scalar(2.8f));

        cv::rectangle(frame.bgr, cv::Rect(35, 65, 70, 105), cv::Scalar(40, 55, 225), cv::FILLED);
        cv::rectangle(frame.depth, cv::Rect(35, 65, 70, 105), cv::Scalar(1.45f), cv::FILLED);
        cv::circle(frame.bgr, cv::Point(225, 105), 43, cv::Scalar(70, 200, 65), cv::FILLED);
        cv::circle(frame.depth, cv::Point(225, 105), 43, cv::Scalar(1.85f), cv::FILLED);
        const std::vector<cv::Point> triangle = {{140, 165}, {195, 215}, {110, 215}};
        cv::fillConvexPoly(frame.bgr, triangle, cv::Scalar(225, 115, 35));
        cv::fillConvexPoly(frame.depth, triangle, cv::Scalar(2.1f));

        // 纹理只改变颜色，不改变几何形状，使 Canny 和 ORB 能观察到不同的特征。
        cv::RNG random(12345);
        for (int i = 0; i < 75; ++i)
        {
            const cv::Point point(random.uniform(12, 308), random.uniform(12, 228));
            cv::rectangle(
                frame.bgr, cv::Rect(point.x, point.y, 5, 5), cv::Scalar(215, 215, 215), 1);
        }

        for (int row = 0; row < frame.depth.rows; ++row)
        {
            for (int column = 0; column < frame.depth.cols; ++column)
            {
                frame.depth.at<float>(row, column) += static_cast<float>(random.gaussian(.012));
            }
        }
        frame.depth(cv::Rect(55, 95, 7, 7)).setTo(0.0f);
        return frame;
    }

    RgbdFrame readRgbd(const QString& colorFile,
        const QString& depthFile,
        const CameraIntrinsics& camera,
        double unitsPerMeter)
    {
        cvRequire(std::isfinite(unitsPerMeter) && unitsPerMeter > 0.0,
            "Depth units per meter must be positive.");
        RgbdFrame frame;
        frame.camera = camera;
        frame.bgr = decodeImage(colorFile, cv::IMREAD_COLOR);
        const cv::Mat raw = decodeImage(depthFile, cv::IMREAD_UNCHANGED);

        if (raw.type() == CV_16UC1)
        {
            raw.convertTo(frame.depth, CV_32F, 1.0 / unitsPerMeter);
        }
        else if (raw.type() == CV_32FC1)
        {
            frame.depth = raw.clone();
        }
        else
        {
            throw std::runtime_error(
                "Depth image must be single-channel 16-bit unsigned or 32-bit float.");
        }

        for (int row = 0; row < frame.depth.rows; ++row)
        {
            for (int column = 0; column < frame.depth.cols; ++column)
            {
                float& depth = frame.depth.at<float>(row, column);
                if (!std::isfinite(depth) || depth < 0.0f)
                {
                    depth = 0.0f;
                }
            }
        }
        validateFrame(frame);
        cvRequire(
            cv::countNonZero(frame.depth > 0.0f) > 0, "Depth image has no valid depth values.");
        return frame;
    }

    void writeCvImage(const QString& filename, const cv::Mat& image)
    {
        cvRequire(!image.empty(), "No image to export.");
        cvRequire(QFileInfo(filename).suffix().compare("png", Qt::CaseInsensitive) == 0,
            "Image export requires a .png filename.");
        std::vector<unsigned char> buffer;
        cvRequire(cv::imencode(".png", image, buffer), "OpenCV PNG encoding failed.");
        QSaveFile file(filename);
        cvRequire(file.open(QIODevice::WriteOnly), "Cannot open image destination.");
        cvRequire(file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size()) ==
                    static_cast<qint64>(buffer.size()) &&
                file.commit(),
            "Cannot save PNG image.");
    }

    void writeDepthPng(const QString& filename, const cv::Mat& depth)
    {
        cvRequire(!depth.empty() && depth.type() == CV_32FC1, "No float depth image to export.");
        double maximum = 0.0;
        cv::minMaxLoc(depth, nullptr, &maximum);
        cvRequire(
            maximum <= 65.535, "16-bit millimeter PNG cannot represent depth above 65.535 meters.");
        cv::Mat millimeters;
        depth.convertTo(millimeters, CV_16U, 1000.0);
        writeCvImage(filename, millimeters);
    }

    cv::Mat depthPreview(const cv::Mat& depth)
    {
        cv::Mat valid = depth > 0.0f;
        cv::Mat grayscale(depth.size(), CV_8UC1, cv::Scalar(0));
        if (cv::countNonZero(valid) > 0)
        {
            double minimum = 0.0;
            double maximum = 0.0;
            cv::minMaxLoc(depth, &minimum, &maximum, nullptr, nullptr, valid);
            const double span = std::max(.001, maximum - minimum);
            depth.convertTo(grayscale, CV_8U, 240.0 / span, -minimum * 240.0 / span + 10.0);
        }
        cv::Mat color;
        cv::applyColorMap(grayscale, color, cv::COLORMAP_TURBO);
        color.setTo(cv::Scalar(16, 24, 36), ~valid);
        return color;
    }

    Result backProject(const RgbdFrame& frame, const RgbdOptions& options, const cv::Mat& mask)
    {
        validateFrame(frame);
        cvRequire(options.stride >= 1 && options.stride <= 32 &&
                std::isfinite(options.minimumDepth) && std::isfinite(options.maximumDepth) &&
                options.minimumDepth > 0.0 && options.minimumDepth < options.maximumDepth,
            "Invalid pixel stride or depth range.");
        cvRequire(mask.empty() || (mask.type() == CV_8UC1 && mask.size() == frame.depth.size()),
            "Pixel mask must be 8-bit and have the same size as depth.");
        Result result;
        for (int row = 0; row < frame.depth.rows; row += options.stride)
        {
            for (int column = 0; column < frame.depth.cols; column += options.stride)
            {
                const float z = frame.depth.at<float>(row, column);
                if (!std::isfinite(z) || z < options.minimumDepth || z > options.maximumDepth ||
                    (!mask.empty() && mask.at<unsigned char>(row, column) == 0))
                {
                    continue;
                }

                // 反投影保留相机坐标系，不把深度当作欧氏距离，也不隐式翻转 Y 轴。
                const float x =
                    static_cast<float>((column - frame.camera.cx) * z / frame.camera.fx);
                const float y = static_cast<float>((row - frame.camera.cy) * z / frame.camera.fy);
                result.cloud->push_back(Point(x, y, z));
                const auto color = frame.bgr.at<cv::Vec3b>(row, column);
                result.colors.push_back({color[2], color[1], color[0]});
                cvRequire(result.cloud->size() <= 500000,
                    "More than 500,000 points: increase the pixel stride before conversion.");
            }
        }
        return result;
    }

    CvResult executeCv(CvOperation operation,
        const RgbdFrame& frame,
        const Values& p,
        const RgbdOptions& options,
        const Result& sourceCloud)
    {
        // OpenCL 的使用开关与调用线程有关；后台任务也明确选择 CPU 路径。
        cv::ocl::setUseOpenCL(false);
        validateFrame(frame);
        validateParameters(operation, p);
        CvResult result;
        result.image = frame.bgr.clone();
        result.depth = frame.depth.clone();
        RgbdFrame working = frame;

        switch (operation)
        {
            case CvOperation::BackProject:
                result.detail = "RGB-D 反投影完成；像素颜色与三维点保持一一对应。";
                break;

            case CvOperation::MedianDepth:
            {
                const int kernel = static_cast<int>(p[0]);
                cvRequire(kernel == 3 || kernel == 5, "Float median kernel must be 3 or 5.");
                cv::medianBlur(frame.depth, result.depth, kernel);
                result.depth.setTo(0.0f, frame.depth <= 0.0f);
                result.image = depthPreview(result.depth);
                result.detail = "中值滤波后的深度重新生成点云；无效深度保持为 0。";
                break;
            }

            case CvOperation::BilateralDepth:
            {
                const int diameter = static_cast<int>(p[0]);
                cvRequire(diameter % 2 == 1, "Bilateral diameter must be odd.");
                const cv::Mat holes = frame.depth <= 0.0f;
                cv::Mat filled;
                if (cv::countNonZero(holes) > 0)
                {
                    cvRequire(
                        cv::countNonZero(holes) < frame.depth.total(), "No valid depth to smooth.");
                    cv::inpaint(frame.depth, holes, filled, 3.0, cv::INPAINT_NS);
                }
                else
                {
                    filled = frame.depth;
                }
                cv::bilateralFilter(filled, result.depth, diameter, p[1], p[2]);
                result.depth.setTo(0.0f, holes);
                result.image = depthPreview(result.depth);
                result.detail = "双边平滑完成；深度 sigma 使用米，原无效像素未被转换成三维点。";
                break;
            }

            case CvOperation::DepthRange:
                result.mask = foregroundMask(frame, p[0], p[1]);
                cv::cvtColor(result.mask, result.image, cv::COLOR_GRAY2BGR);
                break;

            case CvOperation::Morphology:
            {
                const int size = static_cast<int>(p[2]);
                cvRequire(size % 2 == 1, "Morphology kernel must be odd.");
                result.mask = foregroundMask(frame, p[0], p[1]);
                const auto kernel =
                    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
                cv::morphologyEx(result.mask, result.mask, cv::MORPH_OPEN, kernel);
                cv::morphologyEx(result.mask, result.mask, cv::MORPH_CLOSE, kernel);
                cv::cvtColor(result.mask, result.image, cv::COLOR_GRAY2BGR);
                break;
            }

            case CvOperation::Canny:
            {
                cvRequire(p[0] < p[1], "Canny low threshold must be below high threshold.");
                cv::Mat gray;
                cv::cvtColor(frame.bgr, gray, cv::COLOR_BGR2GRAY);
                cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.0);
                cv::Canny(gray, result.mask, p[0], p[1]);
                cv::cvtColor(result.mask, result.image, cv::COLOR_GRAY2BGR);
                result.detail = "颜色边缘已关联有效深度；纹理边缘不一定对应几何边界。";
                break;
            }

            case CvOperation::HsvMask:
            {
                cv::Mat hsv;
                cv::cvtColor(frame.bgr, hsv, cv::COLOR_BGR2HSV);
                if (p[0] <= p[1])
                {
                    cv::inRange(
                        hsv, cv::Scalar(p[0], p[2], p[3]), cv::Scalar(p[1], 255, 255), result.mask);
                }
                else
                {
                    cv::Mat lower;
                    cv::Mat upper;
                    cv::inRange(hsv, cv::Scalar(0, p[2], p[3]), cv::Scalar(p[1], 255, 255), lower);
                    cv::inRange(
                        hsv, cv::Scalar(p[0], p[2], p[3]), cv::Scalar(179, 255, 255), upper);
                    cv::bitwise_or(lower, upper, result.mask);
                }
                result.image.setTo(cv::Scalar(24, 24, 24));
                frame.bgr.copyTo(result.image, result.mask);
                break;
            }

            case CvOperation::Contours:
            {
                auto mask = foregroundMask(frame, p[0], p[1]);
                cv::morphologyEx(mask,
                    mask,
                    cv::MORPH_OPEN,
                    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                result.mask = cv::Mat::zeros(frame.depth.size(), CV_8UC1);
                int count = 0;
                QStringList measurements;

                for (std::size_t i = 0; i < contours.size(); ++i)
                {
                    if (cv::contourArea(contours[i]) < p[2])
                    {
                        continue;
                    }
                    cvRequire(++count <= 64, "More than 64 contours: raise the minimum area.");
                    cv::Mat objectMask = cv::Mat::zeros(frame.depth.size(), CV_8UC1);
                    cv::drawContours(
                        objectMask, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
                    cv::bitwise_or(result.mask, objectMask, result.mask);
                    cv::rectangle(
                        result.image, cv::boundingRect(contours[i]), cv::Scalar(0, 235, 255), 2);

                    const auto object = backProject(frame, options, objectMask);
                    if (!object.cloud->empty())
                    {
                        Eigen::Vector3f low =
                            Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
                        Eigen::Vector3f high = -low;
                        for (const auto& point : *object.cloud)
                        {
                            low = low.cwiseMin(point.getVector3fMap());
                            high = high.cwiseMax(point.getVector3fMap());
                        }
                        const Eigen::Vector3f size = high - low;
                        measurements << QString("目标 %1：%2 点；XYZ 轴向范围 %3 × %4 × %5 米")
                                            .arg(count)
                                            .arg(object.cloud->size())
                                            .arg(size.x(), 0, 'f', 3)
                                            .arg(size.y(), 0, 'f', 3)
                                            .arg(size.z(), 0, 'f', 3);
                    }
                }
                result.detail =
                    QString("有效二维轮廓 %1 个。\n%2").arg(count).arg(measurements.join('\n'));
                break;
            }

            case CvOperation::Orb:
            {
                cv::Mat gray;
                cv::cvtColor(frame.bgr, gray, cv::COLOR_BGR2GRAY);
                const auto detector = cv::ORB::create(static_cast<int>(p[0]),
                    1.2f,
                    8,
                    31,
                    0,
                    2,
                    cv::ORB::HARRIS_SCORE,
                    31,
                    static_cast<int>(p[1]));
                std::vector<cv::KeyPoint> keypoints;
                cv::Mat descriptors;
                detector->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);
                cv::drawKeypoints(frame.bgr, keypoints, result.image, cv::Scalar(0, 220, 255));
                result.mask = cv::Mat::zeros(frame.depth.size(), CV_8UC1);
                for (const auto& keypoint : keypoints)
                {
                    const int x = cvRound(keypoint.pt.x);
                    const int y = cvRound(keypoint.pt.y);
                    if (x >= 0 && y >= 0 && x < frame.depth.cols && y < frame.depth.rows)
                    {
                        result.mask.at<unsigned char>(y, x) = 255;
                    }
                }
                // 特征点按原像素定位，不使用全局像素步长，否则会漏掉不在采样网格上的特征。
                RgbdOptions fullResolution = options;
                fullResolution.stride = 1;
                result.points = backProject(frame, fullResolution, result.mask);
                result.detail = QString("ORB 关键点 %1；描述子 %2 × %3 字节；有效三维像素 %4。")
                                    .arg(keypoints.size())
                                    .arg(descriptors.rows)
                                    .arg(descriptors.cols)
                                    .arg(result.points.cloud->size());
                break;
            }

            case CvOperation::ProjectCloud:
            {
                cvRequire(sourceCloud.cloud && !sourceCloud.cloud->empty(),
                    "PCL workspace has no cloud to project.");
                result.depth = cv::Mat::zeros(frame.depth.size(), CV_32FC1);
                cv::Mat colors = cv::Mat::zeros(frame.bgr.size(), CV_8UC3);
                for (std::size_t i = 0; i < sourceCloud.cloud->size(); ++i)
                {
                    const auto& point = (*sourceCloud.cloud)[i];
                    if (!pcl::isFinite(point) || point.z < options.minimumDepth ||
                        point.z > options.maximumDepth)
                    {
                        continue;
                    }
                    const double u = frame.camera.fx * point.x / point.z + frame.camera.cx;
                    const double v = frame.camera.fy * point.y / point.z + frame.camera.cy;
                    if (u < -.5 || v < -.5 || u >= frame.depth.cols - .5 ||
                        v >= frame.depth.rows - .5)
                    {
                        continue;
                    }
                    const int x = cvRound(u);
                    const int y = cvRound(v);
                    float& nearest = result.depth.at<float>(y, x);
                    if (nearest == 0.0f || point.z < nearest)
                    {
                        nearest = point.z;
                        const Color color = sourceCloud.colors.size() == sourceCloud.cloud->size()
                            ? sourceCloud.colors[i]
                            : Color{};
                        colors.at<cv::Vec3b>(y, x) = cv::Vec3b(color.b, color.g, color.r);
                    }
                }
                RgbdFrame projected{colors, result.depth, frame.camera};
                RgbdOptions perPixel = options;
                perPixel.stride = 1;
                result.points = backProject(projected, perPixel);
                result.image = depthPreview(result.depth);
                result.detail =
                    QString("PCL 输入 %1 点 → %2 个可见像素；同像素采用最近深度，空洞保持为 0。")
                        .arg(sourceCloud.cloud->size())
                        .arg(result.points.cloud->size());
                break;
            }
        }

        if (!result.mask.empty())
        {
            // 导出的深度与选中像素一致；被图像掩膜排除的位置使用无效值 0。
            result.depth.setTo(0.0f, ~result.mask);
        }

        if (operation != CvOperation::Orb && operation != CvOperation::ProjectCloud)
        {
            working.depth = result.depth;
            result.points = backProject(working, options, result.mask);
        }
        result.points.detail = result.detail;
        return result;
    }
} // namespace demo
