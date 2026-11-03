// 先包含 PCL，再包含 Qt UI 头文件，减少第三方头文件与 Qt 宏的相互影响。
#include <pcl/visualization/pcl_visualizer.h>
#include "MainWindow.h"
#include "SelfTest.h"
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <QApplication>
#include <QFont>
#include <QFontDatabase>
#include <QFile>
#include <QIcon>
#include <QMessageBox>
#include <QThread>
#include <cstring>

int main(int argc, char* argv[])
{
    // 教学程序默认使用 CPU，避免依赖特定 OpenCL 设备和额外的并行插件。
    // 保留警告和错误，隐藏 OpenCV 调试版可选插件探测产生的 INFO 输出。
    cv::ocl::setUseOpenCL(false);
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    bool selfTest = false;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--self-test") == 0)
        {
            selfTest = true;
        }
    }

    // 自动验证不弹出窗口：仍使用真实的 Qt 控件、事件与绘制链路。
    if (selfTest)
    {
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }

    QApplication app(argc, argv);
    app.setApplicationName("PCLDemo");
    app.setOrganizationName("PCLLearning");
    app.setWindowIcon(QIcon(":/icons/pcldemo.ico"));

    QString fontFamily = "Microsoft YaHei UI";
    if (selfTest)
    {
        // Windows 的 offscreen 插件未必枚举系统字体。只在离屏验证中注册本机字体；
        // 不复制或分发字体文件，正常桌面启动仍使用系统字体数据库。
        QFile fontFile(qEnvironmentVariable("WINDIR", "C:/Windows") + "/Fonts/msyh.ttc");
        if (fontFile.open(QIODevice::ReadOnly))
        {
            int id = QFontDatabase::addApplicationFontFromData(fontFile.readAll());
            auto families = QFontDatabase::applicationFontFamilies(id);
            if (!families.isEmpty())
            {
                fontFamily = families.first();
            }
        }
    }

    app.setFont(QFont(fontFamily, 9));

    const auto arguments = app.arguments();
    if (selfTest)
    {
        int reportIndex = arguments.indexOf("--report");
        QString report = reportIndex >= 0 && reportIndex + 1 < arguments.size()
            ? arguments[reportIndex + 1]
            : QApplication::applicationDirPath() + "/self-test.txt";
        return demo::runSelfTest(report);
    }

    int nativeIndex = arguments.indexOf("--native-viewer");
    if (nativeIndex >= 0 && nativeIndex + 1 < arguments.size())
    {
        try
        {
            auto cloud = demo::readCloud(arguments[nativeIndex + 1]);
            pcl::visualization::PCLVisualizer viewer("PCL Demo - native viewer");
            viewer.setBackgroundColor(.055, .085, .13);
            pcl::visualization::PointCloudColorHandlerCustom<demo::Point> color(
                cloud, 57, 189, 227);
            viewer.addPointCloud<demo::Point>(cloud, color, "cloud");
            viewer.setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
            viewer.addCoordinateSystem(.3);
            viewer.initCameraParameters();
            viewer.resetCamera();

            // 独立进程中由 PCL / VTK 管理交互，不向 Qt 主窗口传递 VTK 对象。
            while (!viewer.wasStopped())
            {
                viewer.spinOnce(16);
                QThread::msleep(8);
            }
            return 0;
        }
        catch (const std::exception& e)
        {
            QMessageBox::critical(nullptr, "PCL 查看器", QString::fromUtf8(e.what()));
            return 1;
        }
    }

    demo::MainWindow window;
    window.show();
    return app.exec();
}
