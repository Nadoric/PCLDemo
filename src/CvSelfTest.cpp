#include "SelfTest.h"
#include "MainWindow.h"
#include "RgbdAlgorithms.h"
#include "RgbdPanel.h"
#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QThread>
#include <cmath>
#include <stdexcept>

namespace demo
{
    namespace
    {
        void expect(bool condition, const char* message)
        {
            if (!condition)
            {
                throw std::runtime_error(message);
            }
        }

        void waitForCv(MainWindow& window)
        {
            QElapsedTimer timer;
            timer.start();
            while (window.isBusy() && timer.elapsed() < 60000)
            {
                QApplication::processEvents(QEventLoop::AllEvents, 20);
                QThread::msleep(5);
            }
            expect(!window.isBusy(), "OpenCV worker timeout");
            QApplication::processEvents();
        }
    } // namespace

    void runCvSelfTests(const TestCase& test, const QString& reportPath)
    {
        const auto frame = makeRgbdExample();
        const RgbdOptions options;

        test("RGB-D geometry / BGR-RGB correspondence / round-trip projection",
            []
            {
                RgbdFrame tiny;
                tiny.camera = {2.0, 2.0, 0.0, 0.0};
                tiny.bgr = cv::Mat(2, 2, CV_8UC3, cv::Scalar(10, 20, 30));
                tiny.depth = cv::Mat::zeros(2, 2, CV_32FC1);
                tiny.depth.at<float>(0, 0) = 1.f;
                tiny.depth.at<float>(0, 1) = 2.f;
                tiny.depth.at<float>(1, 1) = 4.f;
                RgbdOptions sampling;
                sampling.stride = 1;
                const auto cloud = backProject(tiny, sampling);
                expect(cloud.cloud->size() == 3, "Invalid depth generated points");
                expect(std::abs((*cloud.cloud)[1].x - 1.f) < .00001f &&
                        std::abs((*cloud.cloud)[1].z - 2.f) < .00001f,
                    "Pinhole geometry mismatch");
                expect(
                    cloud.colors[1].r == 30 && cloud.colors[1].g == 20 && cloud.colors[1].b == 10,
                    "BGR / RGB mismatch");
                const auto result = executeCv(CvOperation::ProjectCloud, tiny, {}, sampling, cloud);
                expect(result.points.cloud->size() == 3 && result.depth.at<float>(1, 1) == 4.f,
                    "Projection round trip mismatch");
            });

        test("All 10 OpenCV linked algorithms produce valid images and colored clouds",
            [&]
            {
                const Result source = backProject(frame, options);
                for (const auto& specification : cvSpecifications())
                {
                    const auto result = executeCv(specification.operation,
                        frame,
                        cvDefaults(specification.operation),
                        options,
                        source);
                    expect(!result.image.empty() && result.image.size() == frame.bgr.size(),
                        "Algorithm image mismatch");
                    expect(!result.points.cloud->empty() &&
                            result.points.colors.size() == result.points.cloud->size(),
                        "Point/color mismatch");
                }
            });

        test("Depth smoothing reduces planar noise while preserving invalid pixels",
            [&]
            {
                const cv::Rect planar(40, 70, 40, 20);
                cv::Scalar originalMean;
                cv::Scalar originalDeviation;
                cv::meanStdDev(frame.depth(planar), originalMean, originalDeviation);
                for (auto operation : {CvOperation::MedianDepth, CvOperation::BilateralDepth})
                {
                    const auto result = executeCv(operation, frame, cvDefaults(operation), options);
                    cv::Scalar mean;
                    cv::Scalar deviation;
                    cv::meanStdDev(result.depth(planar), mean, deviation);
                    expect(
                        deviation[0] < originalDeviation[0] * .8, "Smoothing did not reduce noise");
                    expect(result.depth.at<float>(96, 56) == 0.f, "Invalid pixel was replaced");
                    expect(std::abs(mean[0] - 1.45) < .01, "Smoothing shifted the surface");
                }
            });

        test("Depth thresholds / morphology reject background in XYZ and exported depth",
            [&]
            {
                for (auto operation : {CvOperation::DepthRange, CvOperation::Morphology})
                {
                    const auto result = executeCv(operation, frame, cvDefaults(operation), options);
                    expect(result.points.cloud->size() > 1000 && result.points.cloud->size() < 6000,
                        "Foreground count mismatch");
                    for (const auto& point : *result.points.cloud)
                    {
                        expect(point.z >= .2f && point.z <= 2.3f, "Depth mask retained background");
                    }
                    expect(
                        result.depth.at<float>(0, 0) == 0.f, "Masked export retained background");
                }
            });

        test("HSV hue wraparound selects red three-dimensional points",
            [&]
            {
                const auto result = executeCv(
                    CvOperation::HsvMask, frame, cvDefaults(CvOperation::HsvMask), options);
                expect(result.points.cloud->size() > 1000, "No red object selected");
                for (const auto color : result.points.colors)
                {
                    expect(
                        color.r > 2 * color.g && color.r > 2 * color.b, "Non-red point retained");
                }
            });

        test("Canny edges and ORB features associate with valid depth",
            [&]
            {
                const auto edges =
                    executeCv(CvOperation::Canny, frame, cvDefaults(CvOperation::Canny), options);
                expect(cv::countNonZero(edges.mask) > 100 && !edges.points.cloud->empty(),
                    "Missing edges");
                const auto features =
                    executeCv(CvOperation::Orb, frame, cvDefaults(CvOperation::Orb), options);
                expect(features.points.cloud->size() > 20 && features.points.cloud->size() <= 400,
                    "ORB feature count mismatch");
                expect(features.detail.contains("32 字节"), "ORB descriptor width mismatch");
            });

        test("Contours detect three targets and report XYZ ranges",
            [&]
            {
                const auto result = executeCv(
                    CvOperation::Contours, frame, cvDefaults(CvOperation::Contours), options);
                expect(result.detail.contains("轮廓 3 个") && result.detail.contains("XYZ"),
                    "Contour count or measurement output mismatch");
            });

        test("Projection z-buffer retains nearest point and rejects invisible points",
            [&]
            {
                Result source;
                source.cloud->push_back(Point(0.f, 0.f, 2.f));
                source.cloud->push_back(Point(0.f, 0.f, 1.f));
                source.cloud->push_back(Point(1000.f, 0.f, 1.f));
                source.cloud->push_back(Point(0.f, 0.f, -1.f));
                const auto result =
                    executeCv(CvOperation::ProjectCloud, frame, {}, options, source);
                expect(
                    result.points.cloud->size() == 1, "Invisible or occluded points were retained");
                expect(result.depth.at<float>(cvRound(frame.camera.cy), cvRound(frame.camera.cx)) ==
                        1.f,
                    "Z-buffer did not retain nearest point");
            });

        test("Unicode RGB / millimeter uint16 PNG round trip and dimension validation",
            [&]
            {
                QTemporaryDir directory(
                    QCoreApplication::applicationDirPath() + "/rgbd-test-XXXXXX");
                expect(directory.isValid(), "Cannot create test folder");
                const auto rgb = directory.filePath("彩色图像.png");
                const auto depth = directory.filePath("毫米深度.png");
                writeCvImage(rgb, frame.bgr);
                writeDepthPng(depth, frame.depth);
                const auto loaded = readRgbd(rgb, depth, frame.camera, 1000.0);
                expect(cv::norm(frame.bgr, loaded.bgr, cv::NORM_INF) == 0.0,
                    "RGB PNG round trip mismatch");
                expect(cv::norm(frame.depth, loaded.depth, cv::NORM_INF) <= .00051,
                    "Depth units or quantization mismatch");
                writeCvImage(depth, cv::Mat(10, 10, CV_16UC1, cv::Scalar(1000)));
                bool rejected = false;
                try
                {
                    readRgbd(rgb, depth, frame.camera, 1000.0);
                }
                catch (...)
                {
                    rejected = true;
                }
                expect(rejected, "Mismatched image sizes were accepted");
            });

        test("Invalid camera, kernel and missing projection source are rejected",
            [&]
            {
                auto invalid = frame;
                invalid.camera.fx = 0;
                bool rejected = false;
                try
                {
                    backProject(invalid, options);
                }
                catch (...)
                {
                    rejected = true;
                }
                expect(rejected, "Zero focal length accepted");
                rejected = false;
                try
                {
                    executeCv(CvOperation::MedianDepth, frame, {4, 0, 0, 0}, options);
                }
                catch (...)
                {
                    rejected = true;
                }
                expect(rejected, "Even median kernel accepted");
                rejected = false;
                try
                {
                    executeCv(CvOperation::ProjectCloud, frame, {}, options);
                }
                catch (...)
                {
                    rejected = true;
                }
                expect(rejected, "Projection accepted missing source");
            });

        test("Qt OpenCV/PCL: process, transfer, filter, reproject and failure retention",
            [&]
            {
                MainWindow window;
                window.resize(1640, 1060);
                window.show();
                auto panel = window.rgbdPanel();
                auto tabs = window.findChild<QTabWidget*>("workspaceTabs");
                auto algorithms = window.findChild<QComboBox*>("cvAlgorithmList");
                auto run = window.findChild<QPushButton*>("cvRunButton");
                auto send = window.findChild<QPushButton*>("sendRgbdToPcl");
                expect(panel && tabs && algorithms && run && send && algorithms->count() == 10,
                    "OpenCV UI controls missing");
                tabs->setCurrentIndex(1);
                algorithms->setCurrentIndex(6);
                run->click();
                waitForCv(window);
                const auto selectedCount = panel->currentResult().points.cloud->size();
                expect(selectedCount > 1000 && selectedCount < 2500, "UI HSV processing failed");
                send->click();
                expect(tabs->currentIndex() == 0 &&
                        window.currentResult().cloud->size() == selectedCount,
                    "Point cloud transfer to PCL failed");
                expect(
                    window.currentResult().colors.size() == selectedCount, "Transfer lost colors");
                window.selectOperation(Operation::Voxel);
                window.executeSelected();
                waitForCv(window);
                expect(
                    window.currentResult().cloud->size() < selectedCount, "PCL filtering failed");
                tabs->setCurrentIndex(1);
                algorithms->setCurrentIndex(9);
                run->click();
                waitForCv(window);
                expect(!panel->currentResult().points.cloud->empty() &&
                        panel->currentResult().detail.contains("PCL 输入"),
                    "Reverse projection failed");
                const auto previous = panel->currentResult().points.cloud->size();
                algorithms->setCurrentIndex(1);
                window.findChild<QDoubleSpinBox*>("cvParameter0")->setValue(4);
                run->click();
                waitForCv(window);
                expect(panel->currentResult().points.cloud->size() == previous &&
                        window.findChild<QPlainTextEdit*>("cvLogView")
                            ->toPlainText()
                            .contains("失败"),
                    "Failed operation changed state");
                algorithms->setCurrentIndex(7);
                run->click();
                waitForCv(window);
                const auto screenshot =
                    QFileInfo(reportPath).absolutePath() + "/OpenCV-PCL-preview.png";
                QDir().mkpath(QFileInfo(screenshot).absolutePath());
                QApplication::processEvents();
                expect(window.grab().save(screenshot), "OpenCV UI screenshot save failed");
                window.close();
                QApplication::processEvents();
            });
    }
} // namespace demo
