#pragma once

#include "Algorithms.h"
#include <opencv2/core.hpp>

namespace demo
{
    struct CameraIntrinsics
    {
        double fx = 285.0;
        double fy = 285.0;
        double cx = 159.5;
        double cy = 119.5;
    };

    struct RgbdFrame
    {
        cv::Mat bgr;   // CV_8UC3，OpenCV 默认的 BGR 顺序。
        cv::Mat depth; // CV_32FC1，以米为单位；0 表示无效深度。
        CameraIntrinsics camera;
    };

    enum class CvOperation
    {
        BackProject,
        MedianDepth,
        BilateralDepth,
        DepthRange,
        Morphology,
        Canny,
        HsvMask,
        Contours,
        Orb,
        ProjectCloud
    };

    struct CvSpecification
    {
        CvOperation operation;
        QString title;
        QString api;
        QString description;
        std::vector<Parameter> parameters;
    };

    struct RgbdOptions
    {
        int stride = 2;
        double minimumDepth = 0.1;
        double maximumDepth = 10.0;
    };

    struct CvResult
    {
        Result points;
        cv::Mat image;
        cv::Mat depth;
        cv::Mat mask;
        QString detail;
    };

    const std::vector<CvSpecification>& cvSpecifications();
    Values cvDefaults(CvOperation operation);
    RgbdFrame makeRgbdExample();

    // 两张图必须已经配准到同一个像素坐标系；本例不推断相机外参。
    // 16 位无符号 PNG/TIFF 使用 unitsPerMeter 换算；32 位浮点深度直接解释为米。
    RgbdFrame readRgbd(const QString& colorFile,
        const QString& depthFile,
        const CameraIntrinsics& camera,
        double unitsPerMeter);

    void writeCvImage(const QString& filename, const cv::Mat& image);
    void writeDepthPng(const QString& filename, const cv::Mat& depth);
    cv::Mat depthPreview(const cv::Mat& depth);

    // mask 对二维像素进行选择；XYZ 和辅助颜色按相同顺序输出，避免 2D/3D 对应错位。
    Result backProject(
        const RgbdFrame& frame, const RgbdOptions& options, const cv::Mat& mask = {});

    CvResult executeCv(CvOperation operation,
        const RgbdFrame& frame,
        const Values& parameters,
        const RgbdOptions& options,
        const Result& sourceCloud = {});
} // namespace demo
