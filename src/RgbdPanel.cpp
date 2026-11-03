#include "RgbdPanel.h"
#include "PointCloudView.h"

#include <opencv2/imgproc.hpp>
#include <QtConcurrent/QtConcurrentRun>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QSplitter>
#include <QTextBrowser>
#include <QVBoxLayout>
#include <limits>

namespace demo
{
    namespace
    {
        QImage qtImage(const cv::Mat& source)
        {
            if (source.empty())
            {
                return {};
            }

            cv::Mat rgb;
            if (source.channels() == 1)
            {
                cv::cvtColor(source, rgb, cv::COLOR_GRAY2RGB);
            }
            else
            {
                cv::cvtColor(source, rgb, cv::COLOR_BGR2RGB);
            }

            // copy() 让 QImage 拥有像素内存，避免局部 cv::Mat 销毁后界面悬空引用。
            return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
        }
    } // namespace

    class ImagePreview : public QWidget
    {
      public:
        explicit ImagePreview(QString title, QWidget* parent = nullptr)
            : QWidget(parent), title_(std::move(title))
        {
            setMinimumSize(180, 180);
        }

        void setImage(const cv::Mat& image)
        {
            image_ = qtImage(image);
            update();
        }

      protected:
        void paintEvent(QPaintEvent*) override
        {
            QPainter painter(this);
            painter.fillRect(rect(), QColor("#101c2c"));
            painter.setPen(QColor("#dae9f7"));
            painter.drawText(QRect(14, 8, width() - 28, 25), title_);

            if (!image_.isNull())
            {
                const QRect bounds(10, 40, width() - 20, height() - 50);
                const QSize size = image_.size().scaled(bounds.size(), Qt::KeepAspectRatio);
                const QRect destination(
                    bounds.center() - QPoint(size.width() / 2, size.height() / 2), size);
                painter.drawImage(destination, image_);
            }
        }

      private:
        QString title_;
        QImage image_;
    };

    RgbdPanel::RgbdPanel(QWidget* parent) : QWidget(parent)
    {
        buildUi();
        loadExample();
    }

    RgbdPanel::~RgbdPanel()
    {
        if (watcher_)
        {
            watcher_->waitForFinished();
        }
    }

    void RgbdPanel::setCallbacks(
        std::function<Result()> source, std::function<bool(const Result&)> send)
    {
        sourceProvider_ = std::move(source);
        sendToPcl_ = std::move(send);
    }

    void RgbdPanel::setExternalBusy(bool busy)
    {
        externalBusy_ = busy;
        controls_->setEnabled(!busy && !isBusy());
        toolbar_->setEnabled(!busy && !isBusy());
    }

    bool RgbdPanel::isBusy() const
    {
        return watcher_ != nullptr;
    }

    const CvResult& RgbdPanel::currentResult() const
    {
        return result_;
    }

    const RgbdFrame& RgbdPanel::frame() const
    {
        return frame_;
    }

    void RgbdPanel::buildUi()
    {
        auto layout = new QVBoxLayout(this);
        layout->setContentsMargins(18, 12, 18, 12);
        auto title = new QLabel("OpenCV + PCL · 图像与三维联动");
        title->setStyleSheet("font-size:22px;font-weight:700;color:#17324d;");
        layout->addWidget(title);
        layout->addWidget(
            new QLabel("RGB-D 图像 → OpenCV 处理 → 像素反投影 → PCL 算法 → 深度回投影"));

        toolbar_ = new QWidget;
        auto tools = new QHBoxLayout(toolbar_);
        tools->setContentsMargins(0, 0, 0, 0);
        const auto addButton = [this, tools](QString text, const std::function<void()>& callback)
        {
            auto button = new QPushButton(text);
            tools->addWidget(button);
            connect(button, &QPushButton::clicked, this, callback);
            return button;
        };
        addButton("内置 RGB-D 示例",
            [this]
            {
                loadExample();
            });
        addButton("加载 RGB + 深度",
            [this]
            {
                openImages();
            });
        addButton("导出处理图 PNG",
            [this]
            {
                exportImage(false);
            });
        addButton("导出深度 PNG / 毫米",
            [this]
            {
                exportImage(true);
            });
        auto send = addButton("发送至 PCL 工作区",
            [this]
            {
                if (sendToPcl_ && sendToPcl_(result_.points))
                {
                    appendLog("已发送当前结果到 PCL；可继续滤波、分割、重建或配准。");
                }
                else
                {
                    appendLog("发送失败：点云为空或工作区正在计算。");
                }
            });
        send->setObjectName("sendRgbdToPcl");
        tools->addStretch();
        layout->addWidget(toolbar_);

        auto split = new QSplitter(Qt::Horizontal);
        split->setChildrenCollapsible(false);
        auto scroll = new QScrollArea;
        scroll->setWidgetResizable(true);
        scroll->setMinimumWidth(310);
        controls_ = new QWidget;
        auto controls = new QVBoxLayout(controls_);

        algorithms_ = new QComboBox;
        algorithms_->setObjectName("cvAlgorithmList");
        for (const auto& specification : cvSpecifications())
        {
            algorithms_->addItem(specification.title);
        }
        controls->addWidget(algorithms_);

        auto parameterForm = new QFormLayout;
        for (int i = 0; i < 4; ++i)
        {
            parameterLabels_[i] = new QLabel;
            parameters_[i] = new QDoubleSpinBox;
            parameters_[i]->setObjectName(QString("cvParameter%1").arg(i));
            parameters_[i]->setKeyboardTracking(false);
            parameterForm->addRow(parameterLabels_[i], parameters_[i]);
        }
        controls->addLayout(parameterForm);
        auto run = new QPushButton("执行 OpenCV → PCL");
        run->setObjectName("cvRunButton");
        run->setMinimumHeight(40);
        run->setStyleSheet("background:#087fbb;color:white;font-weight:700;");
        connect(run, &QPushButton::clicked, this, &RgbdPanel::executeSelected);
        controls->addWidget(run);

        auto cameraGroup = new QGroupBox("相机内参 / 深度单位");
        auto cameraForm = new QFormLayout(cameraGroup);
        const QStringList names = {"fx / 像素", "fy / 像素", "cx / 像素", "cy / 像素"};
        for (int i = 0; i < 4; ++i)
        {
            camera_[i] = new QDoubleSpinBox;
            camera_[i]->setRange(i < 2 ? .001 : -10000.0, 100000.0);
            camera_[i]->setDecimals(3);
            camera_[i]->setKeyboardTracking(false);
            cameraForm->addRow(names[i], camera_[i]);
        }
        depthUnits_ = new QDoubleSpinBox;
        depthUnits_->setRange(.001, 1000000);
        depthUnits_->setValue(1000);
        depthUnits_->setToolTip(
            "只在加载 16 位深度图时使用；1000 表示文件以毫米为单位。32 位浮点图按米读取。");
        cameraForm->addRow("加载时：深度单位 / 米", depthUnits_);
        controls->addWidget(cameraGroup);

        auto samplingGroup = new QGroupBox("反投影采样 / 有效范围");
        auto sampling = new QFormLayout(samplingGroup);
        stride_ = new QSpinBox;
        stride_->setRange(1, 32);
        stride_->setValue(2);
        stride_->setObjectName("rgbdStride");
        minimumDepth_ = new QDoubleSpinBox;
        minimumDepth_->setRange(.001, 60);
        minimumDepth_->setDecimals(3);
        minimumDepth_->setValue(.1);
        maximumDepth_ = new QDoubleSpinBox;
        maximumDepth_->setRange(.002, 65.535);
        maximumDepth_->setDecimals(3);
        maximumDepth_->setValue(10);
        sampling->addRow("像素步长", stride_);
        sampling->addRow("最小有效深度 / 米", minimumDepth_);
        sampling->addRow("最大有效深度 / 米", maximumDepth_);
        controls->addWidget(samplingGroup);

        help_ = new QTextBrowser;
        help_->setMinimumHeight(190);
        controls->addWidget(help_, 1);
        scroll->setWidget(controls_);
        split->addWidget(scroll);

        auto previews = new QSplitter(Qt::Vertical);
        previews->setChildrenCollapsible(false);
        auto images = new QSplitter(Qt::Horizontal);
        inputImage_ = new ImagePreview("输入 RGB / 已与深度配准");
        outputImage_ = new ImagePreview("处理结果 / 伪彩深度、掩膜或特征");
        images->addWidget(inputImage_);
        images->addWidget(outputImage_);
        previews->addWidget(images);
        cloudView_ = new PointCloudView("OpenCV → PCL 三维预览");
        cloudView_->setObjectName("rgbdCloudView");
        previews->addWidget(cloudView_);
        previews->setSizes({290, 390});
        split->addWidget(previews);
        split->setStretchFactor(1, 1);
        split->setSizes({330, 1180});
        layout->addWidget(split, 1);

        summary_ = new QLabel;
        summary_->setStyleSheet("background:#eaf2f8;padding:8px;color:#32546d;");
        layout->addWidget(summary_);
        log_ = new QPlainTextEdit;
        log_->setObjectName("cvLogView");
        log_->setReadOnly(true);
        log_->setMaximumBlockCount(500);
        log_->setMaximumHeight(100);
        layout->addWidget(log_);
        connect(
            algorithms_, &QComboBox::currentIndexChanged, this, &RgbdPanel::updateSpecification);
        updateSpecification();
    }

    CameraIntrinsics RgbdPanel::intrinsics() const
    {
        return {camera_[0]->value(), camera_[1]->value(), camera_[2]->value(), camera_[3]->value()};
    }

    RgbdOptions RgbdPanel::options() const
    {
        return {stride_->value(), minimumDepth_->value(), maximumDepth_->value()};
    }

    void RgbdPanel::updateSpecification()
    {
        const auto& specification = cvSpecifications().at(algorithms_->currentIndex());
        for (int i = 0; i < 4; ++i)
        {
            const bool visible = i < static_cast<int>(specification.parameters.size());
            parameters_[i]->setVisible(visible);
            parameterLabels_[i]->setVisible(visible);
            if (!visible)
            {
                continue;
            }
            const auto& parameter = specification.parameters[i];
            parameterLabels_[i]->setText(parameter.name);
            parameters_[i]->setDecimals(parameter.decimals);
            parameters_[i]->setRange(parameter.minimum, parameter.maximum);
            parameters_[i]->setSingleStep(parameter.step);
            parameters_[i]->setValue(parameter.value);
        }
        help_->setHtml(QString(
            "<h3>%1</h3><p style='color:#087fbb'>%2</p><p>%3</p>"
            "<p>每次执行从当前加载的原始 RGB-D 图像开始，不累积修改输入图。"
            "仅第 10 项读取 PCL 工作区的当前点云。</p>"
            "<p>颜色在预览中保留。发送后，PCL 算法通常只输出 XYZ，辅助颜色不会自动传播。</p>")
                .arg(specification.title.toHtmlEscaped(),
                    specification.api.toHtmlEscaped(),
                    specification.description.toHtmlEscaped()));
    }

    void RgbdPanel::loadExample()
    {
        if (isBusy() || externalBusy_)
        {
            return;
        }
        frame_ = makeRgbdExample();
        const double values[] = {
            frame_.camera.fx, frame_.camera.fy, frame_.camera.cx, frame_.camera.cy};
        for (int i = 0; i < 4; ++i)
        {
            camera_[i]->setValue(values[i]);
        }
        stride_->setValue(2);
        minimumDepth_->setValue(.1);
        maximumDepth_->setValue(10);
        depthUnits_->setValue(1000);
        algorithms_->setCurrentIndex(0);
        result_ = executeCv(CvOperation::BackProject, frame_, {}, options());
        displayResult();
        appendLog("已生成 320×240 配准 RGB-D 示例，包含三个目标、颜色纹理、深度噪声和小孔洞。");
    }

    void RgbdPanel::displayResult()
    {
        inputImage_->setImage(frame_.bgr);
        outputImage_->setImage(result_.image);
        cloudView_->setResult(result_.points);

        Camera camera;
        camera.pitch = -.65;
        if (!result_.points.cloud->empty())
        {
            Eigen::Vector3f low = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
            Eigen::Vector3f high = -low;
            for (const auto& point : *result_.points.cloud)
            {
                low = low.cwiseMin(point.getVector3fMap());
                high = high.cwiseMax(point.getVector3fMap());
            }
            camera.center = (low + high) * .5f;
            camera.radius = std::max(.001f, (high - low).norm() * .5f);
        }
        cloudView_->setCamera(camera);
        summary_->setText(QString(
            "输入 %1 × %2 像素  |  输出 %3 点  |  相机坐标：X 向右，Y 向下，Z 向前；单位为米")
                .arg(frame_.bgr.cols)
                .arg(frame_.bgr.rows)
                .arg(result_.points.cloud->size()));
    }

    void RgbdPanel::appendLog(const QString& text)
    {
        log_->appendPlainText(QDateTime::currentDateTime().toString("[HH:mm:ss] ") + text);
    }

    void RgbdPanel::startJob(const QString& title, std::function<CvJob()> work)
    {
        if (isBusy() || externalBusy_)
        {
            return;
        }
        appendLog("开始：" + title);
        watcher_ = new QFutureWatcher<CvJob>(this);
        controls_->setEnabled(false);
        toolbar_->setEnabled(false);
        auto watcher = watcher_;
        connect(watcher,
            &QFutureWatcher<CvJob>::finished,
            this,
            [this, watcher, title]
            {
                const auto outcome = watcher->result();
                watcher_ = nullptr;
                watcher->deleteLater();
                setExternalBusy(externalBusy_);
                if (!outcome.error.isEmpty())
                {
                    appendLog("失败：" + outcome.error + "\n原有图像与结果已保留。");
                    return;
                }
                if (outcome.replaceFrame)
                {
                    frame_ = outcome.replacement;
                }
                result_ = outcome.result;
                displayResult();
                appendLog(QString("完成：%1，%2 ms，%3 点。\n%4")
                        .arg(title)
                        .arg(outcome.milliseconds, 0, 'f', 1)
                        .arg(result_.points.cloud->size())
                        .arg(result_.detail));
            });
        watcher->setFuture(QtConcurrent::run(
            [work]
            {
                CvJob job;
                QElapsedTimer timer;
                timer.start();
                try
                {
                    job = work();
                }
                catch (const std::exception& error)
                {
                    job.error = QString::fromUtf8(error.what());
                }
                catch (...)
                {
                    job.error = "未知 OpenCV / PCL 异常。";
                }
                job.milliseconds = timer.nsecsElapsed() / 1000000.0;
                return job;
            }));
    }

    void RgbdPanel::executeSelected()
    {
        if (isBusy() || externalBusy_)
        {
            return;
        }
        const auto specification = cvSpecifications().at(algorithms_->currentIndex());
        Values values{};
        for (std::size_t i = 0; i < specification.parameters.size(); ++i)
        {
            parameters_[i]->interpretText();
            values[i] = parameters_[i]->value();
        }
        for (auto field : camera_)
        {
            field->interpretText();
        }
        RgbdFrame snapshot = frame_;
        snapshot.camera = intrinsics();
        const auto sampling = options();
        const Result source =
            specification.operation == CvOperation::ProjectCloud && sourceProvider_
            ? sourceProvider_()
            : Result{};
        startJob(specification.title,
            [snapshot, sampling, specification, values, source]
            {
                CvJob job;
                job.result = executeCv(specification.operation, snapshot, values, sampling, source);
                return job;
            });
    }

    void RgbdPanel::openImages()
    {
        const auto color = QFileDialog::getOpenFileName(
            this, "打开 RGB 图像", {}, "图像 (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)");
        if (color.isEmpty())
        {
            return;
        }
        const auto depth = QFileDialog::getOpenFileName(
            this, "打开已配准的深度图", {}, "深度图 (*.png *.tif *.tiff *.exr)");
        if (depth.isEmpty())
        {
            return;
        }
        const auto camera = intrinsics();
        const double units = depthUnits_->value();
        const auto sampling = options();
        startJob("加载 RGB-D 图像",
            [color, depth, camera, units, sampling]
            {
                CvJob job;
                job.replacement = readRgbd(color, depth, camera, units);
                job.replaceFrame = true;
                job.result = executeCv(CvOperation::BackProject, job.replacement, {}, sampling);
                job.result.detail +=
                    "\n请确认相机内参与输入图像一致；本例不自动推断焦距或 RGB-D 外参。";
                return job;
            });
    }

    void RgbdPanel::exportImage(bool depth)
    {
        QString filename = QFileDialog::getSaveFileName(this,
            depth ? "导出 16 位深度 PNG，单位毫米" : "导出处理图 PNG",
            depth ? "depth-mm.png" : "processed.png",
            "PNG 图像 (*.png)");
        if (filename.isEmpty())
        {
            return;
        }
        if (QFileInfo(filename).suffix().isEmpty())
        {
            filename += ".png";
        }
        try
        {
            if (depth)
            {
                writeDepthPng(filename, result_.depth);
            }
            else
            {
                writeCvImage(filename, result_.image);
            }
            appendLog("已保存：" + filename);
        }
        catch (const std::exception& error)
        {
            appendLog("导出失败：" + QString::fromUtf8(error.what()));
        }
    }
} // namespace demo
