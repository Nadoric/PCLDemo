#pragma once

#include "RgbdAlgorithms.h"
#include <QFutureWatcher>
#include <QWidget>
#include <functional>

class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QPlainTextEdit;
class QTextBrowser;
class QLabel;

namespace demo
{
    class ImagePreview;
    class PointCloudView;

    struct CvJob
    {
        CvResult result;
        RgbdFrame replacement;
        bool replaceFrame = false;
        QString error;
        double milliseconds = 0.0;
    };

    // 本控件仅使用 Qt 标准信号与 lambda，不声明新信号，因此无需额外 moc 生成规则。
    class RgbdPanel : public QWidget
    {
      public:
        explicit RgbdPanel(QWidget* parent = nullptr);
        ~RgbdPanel() override;

        void setCallbacks(std::function<Result()> source, std::function<bool(const Result&)> send);
        void setExternalBusy(bool busy);
        bool isBusy() const;
        const CvResult& currentResult() const;
        const RgbdFrame& frame() const;
        void loadExample();
        void executeSelected();

      private:
        void buildUi();
        void updateSpecification();
        void openImages();
        void exportImage(bool depth);
        void displayResult();
        void appendLog(const QString& text);
        void startJob(const QString& title, std::function<CvJob()> work);
        CameraIntrinsics intrinsics() const;
        RgbdOptions options() const;

        RgbdFrame frame_;
        CvResult result_;
        QComboBox* algorithms_ = nullptr;
        std::array<QDoubleSpinBox*, 4> parameters_{};
        std::array<QLabel*, 4> parameterLabels_{};
        std::array<QDoubleSpinBox*, 4> camera_{};
        QDoubleSpinBox* depthUnits_ = nullptr;
        QDoubleSpinBox* minimumDepth_ = nullptr;
        QDoubleSpinBox* maximumDepth_ = nullptr;
        QSpinBox* stride_ = nullptr;
        QTextBrowser* help_ = nullptr;
        QPlainTextEdit* log_ = nullptr;
        QLabel* summary_ = nullptr;
        ImagePreview* inputImage_ = nullptr;
        ImagePreview* outputImage_ = nullptr;
        PointCloudView* cloudView_ = nullptr;
        QWidget* controls_ = nullptr;
        QWidget* toolbar_ = nullptr;
        QFutureWatcher<CvJob>* watcher_ = nullptr;
        bool externalBusy_ = false;
        std::function<Result()> sourceProvider_;
        std::function<bool(const Result&)> sendToPcl_;
    };
} // namespace demo
