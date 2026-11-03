#pragma once
#include "Algorithms.h"
#include <QMainWindow>
#include <QFutureWatcher>
#include <functional>

class QListWidget;
class QTextBrowser;
class QPlainTextEdit;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QCloseEvent;

namespace demo
{
    class PointCloudView;
    class RgbdPanel;

    struct JobOutcome
    {
        Result result;
        QString error;
        double milliseconds = 0;
    };

    class MainWindow : public QMainWindow
    {
        Q_OBJECT

      public:
        explicit MainWindow(QWidget* parent = nullptr);
        ~MainWindow() override;

        bool isBusy() const;

        RgbdPanel* rgbdPanel() const
        {
            return rgbdPanel_;
        }

        const Result& currentResult() const
        {
            return current_;
        }

        const Cloud::ConstPtr originalCloud() const
        {
            return original_;
        }

        void loadSample(int kind); // 0=桌面场景，1=球面，2=ICP。也供自动验证调用。
        void selectOperation(Operation operation);
        void executeSelected();
        void undo();
        void reset();

      protected:
        void closeEvent(QCloseEvent*) override;

      private:
        void buildUi();
        void updateSpecification();
        void refresh();
        void fitViews();
        void appendLog(const QString& message);
        void setBusy(bool busy);
        void openCloud(bool target);
        void saveResult(bool mesh);
        void openNativeViewer();
        void runTask(const QString& label,
            std::function<Result()> work,
            std::function<void(const Result&)> success);
        const Specification& specification() const;
        QListWidget* operations_ = nullptr;
        QTextBrowser* help_ = nullptr;
        QPlainTextEdit* log_ = nullptr;
        std::array<QDoubleSpinBox*, 4> parameters_{};
        std::array<QLabel*, 4> parameterLabels_{};
        QLabel* summary_ = nullptr;
        QLabel* parameterTitle_ = nullptr;
        QLabel* targetLabel_ = nullptr;
        QPushButton* runButton_ = nullptr;
        QWidget* operationPanel_ = nullptr;
        PointCloudView* originalView_ = nullptr;
        PointCloudView* resultView_ = nullptr;
        RgbdPanel* rgbdPanel_ = nullptr;
        std::vector<QAction*> changingActions_;
        QFutureWatcher<JobOutcome>* watcher_ = nullptr;
        Cloud::Ptr original_;
        Cloud::Ptr target_;
        Result current_;
        std::vector<Result> history_;
    };
} // namespace demo
