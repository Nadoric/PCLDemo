#include "MainWindow.h"
#include "PointCloudView.h"
#include "RgbdPanel.h"
#include <pcl/common/point_tests.h>
#include <QtConcurrent/QtConcurrentRun>
#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QLabel>
#include <QListWidget>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTextBrowser>
#include <QTabWidget>
#include <QToolBar>
#include <QVBoxLayout>
#include <limits>

namespace demo
{
    MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
    {
        setWindowTitle("OpenCV + PCL Demo · 图像与点云算法实验室");
        resize(1540, 960);
        setMinimumSize(1140, 760);
        buildUi();
        loadSample(0);
    }

    MainWindow::~MainWindow()
    {
        // 后台任务只持有点云快照，不访问窗口；退出前等待，避免工作线程访问已卸载的 PCL。
        if (watcher_)
        {
            watcher_->waitForFinished();
        }
    }

    bool MainWindow::isBusy() const
    {
        return watcher_ != nullptr || (rgbdPanel_ && rgbdPanel_->isBusy());
    }

    void MainWindow::buildUi()
    {
        auto fileMenu = menuBar()->addMenu("文件(&F)");
        auto samples = menuBar()->addMenu("示例(&S)");
        auto viewMenu = menuBar()->addMenu("视图(&V)");
        auto helpMenu = menuBar()->addMenu("帮助(&H)");
        auto toolbar = addToolBar("数据与处理");
        toolbar->setMovable(false);
        toolbar->setToolButtonStyle(Qt::ToolButtonTextOnly);
        auto action = [&](QString title, auto callback, QMenu* menu, bool changing = true)
        {
            auto a = new QAction(title, this);
            connect(a, &QAction::triggered, this, callback);
            toolbar->addAction(a);
            menu->addAction(a);
            if (changing)
            {
                changingActions_.push_back(a);
            }
            return a;
        };
        action(
            "打开点云",
            [this]
            {
                openCloud(false);
            },
            fileMenu)
            ->setShortcut(QKeySequence::Open);
        action(
            "加载 ICP 目标",
            [this]
            {
                openCloud(true);
            },
            fileMenu);
        action(
            "导出点云",
            [this]
            {
                saveResult(false);
            },
            fileMenu)
            ->setShortcut(QKeySequence::Save);
        action(
            "导出网格",
            [this]
            {
                saveResult(true);
            },
            fileMenu);
        toolbar->addSeparator();
        action(
            "桌面示例",
            [this]
            {
                loadSample(0);
            },
            samples);
        action(
            "球面示例",
            [this]
            {
                loadSample(1);
            },
            samples);
        action(
            "ICP 示例",
            [this]
            {
                loadSample(2);
            },
            samples);
        toolbar->addSeparator();
        auto undoAction = action(
            "撤销",
            [this]
            {
                undo();
            },
            fileMenu);
        undoAction->setObjectName("undoAction");
        undoAction->setShortcut(QKeySequence::Undo);
        action(
            "恢复原始",
            [this]
            {
                reset();
            },
            fileMenu);
        action(
            "适应视图",
            [this]
            {
                fitViews();
            },
            viewMenu,
            false)
            ->setShortcut(QKeySequence("F"));
        action(
            "PCL 原生查看器",
            [this]
            {
                openNativeViewer();
            },
            viewMenu);
        auto about = helpMenu->addAction("使用说明");
        connect(about,
            &QAction::triggered,
            this,
            [this]
            {
                QMessageBox::information(this,
                    "PCL Demo 使用说明",
                    "1. 打开 PCD / PLY，或使用三个内置示例。\n"
                    "2. 选择算法，阅读右侧说明并调整参数。\n"
                    "3. 执行后，右侧结果成为下次运算的输入；左侧原始数据保持不变。\n"
                    "4. 可撤销最近 8 步，或恢复原始数据；点云与三角网格分别导出。\n\n"
                    "两侧视图同步：左键旋转、右键平移、滚轮缩放、F 适应、双击恢复角度。\n"
                    "嵌入视图为 Qt 正交投影，显示最多抽样 45,000 点，网格显示线框。\n"
                    "“PCL 原生查看器”另开 PCLVisualizer / VTK 窗口，支持完整点云交互。\n\n"
                    "本项目处理 XYZ 字段；RGB、强度、法线和特征不会写入点云导出。\n"
                    "单位遵循输入坐标。详细参数、学习路线和限制见项目 README.md。");
            });

        auto central = new QWidget(this);
        auto layout = new QVBoxLayout(central);
        layout->setContentsMargins(18, 12, 18, 12);
        layout->setSpacing(12);
        auto title =
            new QLabel("点云算法实验室  <span style='font-size:12px;color:#64819b'>PCL 1.15 · "
                       "C++17 · Qt 6</span>");
        title->setStyleSheet("font-size:23px;font-weight:700;color:#17324d;");
        layout->addWidget(title);
        auto hint =
            new QLabel("加载 / 生成点云  →  选择算法与参数  →  比较结果  →  连续处理 / 导出");
        hint->setStyleSheet("color:#6b8195;padding-bottom:4px;");
        layout->addWidget(hint);
        auto horizontal = new QSplitter(Qt::Horizontal);
        horizontal->setChildrenCollapsible(false);
        operationPanel_ = new QWidget;
        auto panelLayout = new QVBoxLayout(operationPanel_);
        panelLayout->setContentsMargins(0, 0, 8, 0);
        auto heading = new QLabel("算法目录  /  15 个示例");
        heading->setStyleSheet("font-weight:700;color:#38546e;padding:8px 0;");
        panelLayout->addWidget(heading);
        operations_ = new QListWidget;
        operations_->setObjectName("algorithmList");
        operations_->setMinimumWidth(212);
        for (const auto& spec : specifications())
        {
            operations_->addItem(spec.title);
        }
        operations_->setSpacing(2);
        panelLayout->addWidget(operations_, 1);
        parameterTitle_ = new QLabel("参数");
        parameterTitle_->setStyleSheet("font-weight:700;padding-top:8px;");
        panelLayout->addWidget(parameterTitle_);
        auto form = new QFormLayout;
        form->setRowWrapPolicy(QFormLayout::WrapLongRows);
        for (int i = 0; i < 4; ++i)
        {
            parameterLabels_[i] = new QLabel;
            parameters_[i] = new QDoubleSpinBox;
            parameters_[i]->setObjectName(QString("parameter%1").arg(i));
            parameters_[i]->setMinimumWidth(112);
            parameters_[i]->setKeyboardTracking(false);
            form->addRow(parameterLabels_[i], parameters_[i]);
        }
        panelLayout->addLayout(form);
        auto defaults = new QPushButton("恢复此算法的默认参数");
        connect(defaults, &QPushButton::clicked, this, &MainWindow::updateSpecification);
        panelLayout->addWidget(defaults);
        runButton_ = new QPushButton("执行算法");
        runButton_->setObjectName("runButton");
        runButton_->setMinimumHeight(42);
        runButton_->setStyleSheet(
            "QPushButton{background:#087fbb;color:white;border:0;border-radius:7px;font-weight:700;"
            "} "
            "QPushButton:hover{background:#096da1;} QPushButton:disabled{background:#9cb8c8;}");
        connect(runButton_, &QPushButton::clicked, this, &MainWindow::executeSelected);
        panelLayout->addWidget(runButton_);
        auto unitNote =
            new QLabel("距离参数的单位与输入坐标一致。\n连续执行会处理右侧的最新结果。");
        unitNote->setStyleSheet("color:#71869b;font-size:11px;padding-top:3px;");
        panelLayout->addWidget(unitNote);
        horizontal->addWidget(operationPanel_);

        auto middle = new QWidget;
        auto middleLayout = new QVBoxLayout(middle);
        middleLayout->setContentsMargins(4, 0, 4, 0);
        auto viewOptions = new QHBoxLayout;
        auto viewCaption = new QLabel("同步三维视图  /  原始数据与处理结果");
        viewCaption->setStyleSheet("font-weight:700;color:#38546e;");
        viewOptions->addWidget(viewCaption);
        viewOptions->addStretch();
        auto normals = new QCheckBox("法线");
        normals->setChecked(true);
        viewOptions->addWidget(normals);
        auto mesh = new QCheckBox("网格");
        mesh->setChecked(true);
        viewOptions->addWidget(mesh);
        auto pointSize = new QSpinBox;
        pointSize->setRange(1, 6);
        pointSize->setValue(3);
        pointSize->setPrefix("点径 ");
        viewOptions->addWidget(pointSize);
        middleLayout->addLayout(viewOptions);
        auto views = new QSplitter(Qt::Horizontal);
        views->setChildrenCollapsible(false);
        originalView_ = new PointCloudView("原始点云");
        originalView_->setObjectName("originalView");
        resultView_ = new PointCloudView("当前结果");
        resultView_->setObjectName("resultView");
        views->addWidget(originalView_);
        views->addWidget(resultView_);
        views->setSizes({400, 400});
        middleLayout->addWidget(views, 1);
        connect(originalView_,
            &PointCloudView::cameraChanged,
            this,
            [this]
            {
                resultView_->setCamera(originalView_->camera(), true);
            });
        connect(resultView_,
            &PointCloudView::cameraChanged,
            this,
            [this]
            {
                originalView_->setCamera(resultView_->camera(), true);
            });
        connect(normals,
            &QCheckBox::toggled,
            this,
            [this](bool b)
            {
                resultView_->setShowNormals(b);
            });
        connect(mesh,
            &QCheckBox::toggled,
            this,
            [this](bool b)
            {
                resultView_->setShowMesh(b);
            });
        connect(pointSize,
            &QSpinBox::valueChanged,
            this,
            [this](int v)
            {
                originalView_->setPointSize(v);
                resultView_->setPointSize(v);
            });
        summary_ = new QLabel;
        summary_->setObjectName("summaryLabel");
        summary_->setStyleSheet("background:#eaf2f8;border-radius:6px;padding:10px;color:#32546d;");
        middleLayout->addWidget(summary_);
        targetLabel_ = new QLabel("ICP 目标：未加载");
        targetLabel_->setWordWrap(true);
        targetLabel_->setStyleSheet("color:#64819b;");
        middleLayout->addWidget(targetLabel_);
        horizontal->addWidget(middle);

        auto helpPanel = new QWidget;
        auto helpLayout = new QVBoxLayout(helpPanel);
        helpLayout->setContentsMargins(8, 0, 0, 0);
        auto helpTitle = new QLabel("用法说明与实验建议");
        helpTitle->setStyleSheet("font-weight:700;color:#38546e;padding:8px 0;");
        helpLayout->addWidget(helpTitle);
        help_ = new QTextBrowser;
        help_->setObjectName("helpBrowser");
        help_->setMinimumWidth(220);
        help_->setOpenExternalLinks(false);
        helpLayout->addWidget(help_);
        horizontal->addWidget(helpPanel);
        horizontal->setStretchFactor(0, 0);
        horizontal->setStretchFactor(1, 1);
        horizontal->setStretchFactor(2, 0);
        horizontal->setSizes({250, 900, 280});
        auto vertical = new QSplitter(Qt::Vertical);
        vertical->setChildrenCollapsible(false);
        vertical->addWidget(horizontal);
        auto logPanel = new QWidget;
        auto logLayout = new QVBoxLayout(logPanel);
        logLayout->setContentsMargins(0, 0, 0, 0);
        auto logTitleRow = new QHBoxLayout;
        auto logTitle = new QLabel("运行日志  /  点数、耗时、拟合与配准结果");
        logTitle->setStyleSheet("font-weight:700;color:#38546e;");
        logTitleRow->addWidget(logTitle);
        logTitleRow->addStretch();
        auto clear = new QPushButton("清空日志");
        logTitleRow->addWidget(clear);
        logLayout->addLayout(logTitleRow);
        log_ = new QPlainTextEdit;
        log_->setObjectName("logView");
        log_->setReadOnly(true);
        log_->setMaximumBlockCount(1200);
        log_->setMinimumHeight(90);
        logLayout->addWidget(log_);
        connect(clear, &QPushButton::clicked, log_, &QPlainTextEdit::clear);
        vertical->addWidget(logPanel);
        vertical->setSizes({650, 150});
        layout->addWidget(vertical, 1);
        auto tabs = new QTabWidget;
        tabs->setObjectName("workspaceTabs");
        tabs->addTab(central, "PCL 点云工作区");
        rgbdPanel_ = new RgbdPanel;
        tabs->addTab(rgbdPanel_, "OpenCV + PCL 联动");
        rgbdPanel_->setCallbacks(
            [this]
            {
                return current_;
            },
            [this, tabs](const Result& result)
            {
                if (isBusy() || !result.cloud || result.cloud->empty())
                {
                    return false;
                }

                original_ = result.cloud;
                current_ = result;
                target_.reset();
                history_.clear();
                refresh();
                fitViews();
                tabs->setCurrentIndex(0);
                appendLog(QString("已接收 OpenCV 生成的 %1 点；单位为米，坐标为相机坐标系。")
                        .arg(result.cloud->size()));
                return true;
            });
        setCentralWidget(tabs);
        statusBar()->showMessage("就绪 · 完整数据参与计算，嵌入视图仅对显示进行抽样");
        connect(
            operations_, &QListWidget::currentRowChanged, this, &MainWindow::updateSpecification);
        operations_->setCurrentRow(0);
        setStyleSheet(
            "QMainWindow{background:#f4f7fb;} QWidget{font-size:12px;}"
            "QToolBar{background:white;border-bottom:1px solid #dce5ee;spacing:5px;padding:8px;}"
            "QToolButton{padding:7px 10px;border-radius:5px;color:#26455d;} "
            "QToolButton:hover{background:#e4f1fa;}"
            "QListWidget,QTextBrowser,QPlainTextEdit{background:white;border:1px solid "
            "#dce5ee;border-radius:7px;padding:5px;}"
            "QListWidget::item{padding:7px 6px;border-radius:5px;color:#35536a;}"
            "QListWidget::item:selected{background:#def0fb;color:#006caa;}"
            "QPushButton{padding:6px 10px;border:1px solid "
            "#cbdbe7;border-radius:5px;background:white;color:#36566e;}"
            "QPushButton:hover{background:#eaf4fb;}"
            "QDoubleSpinBox,QSpinBox{padding:5px;border:1px solid "
            "#cbdbe7;border-radius:4px;background:white;}"
            "QSplitter::handle{background:#e5edf5;} QStatusBar{color:#607c91;}");
    }

    const Specification& MainWindow::specification() const
    {
        int row = std::clamp(operations_->currentRow(), 0, int(specifications().size()) - 1);
        return specifications()[row];
    }

    void MainWindow::updateSpecification()
    {
        const auto& spec = specification();
        parameterTitle_->setText("运行参数");
        for (int i = 0; i < 4; ++i)
        {
            bool visible = i < int(spec.parameters.size());
            parameters_[i]->setVisible(visible);
            parameterLabels_[i]->setVisible(visible);
            if (!visible)
            {
                continue;
            }
            const auto& p = spec.parameters[i];
            parameterLabels_[i]->setText(p.name);
            parameters_[i]->setDecimals(p.decimals);
            parameters_[i]->setRange(p.minimum, p.maximum);
            parameters_[i]->setSingleStep(p.step);
            parameters_[i]->setValue(p.value);
        }
        help_->setHtml(
            QString("<style>body{color:#3c566e;font-size:12px;} h3{color:#173c5b;} "
                    "p{line-height:1.65;} code{color:#0877af;} </style>"
                    "<h3>%1</h3><p><code>%2</code></p><p>%3</p><h3>建议实验</h3><p>%4</p>"
                    "<h3>数据流</h3><p>左：原始数据<br>右：当前结果 / "
                    "下一次输入<br>撤销：最多 8 步<br>恢复原始：清空处理链</p>"
                    "<h3>源码位置</h3><p>src/Algorithms.cpp<br>execute() "
                    "中的对应分支包含完整调用步骤和中文注释。</p>")
                .arg(spec.title.toHtmlEscaped(),
                    spec.api.toHtmlEscaped(),
                    spec.description.toHtmlEscaped(),
                    spec.recipe.toHtmlEscaped()));
    }

    void MainWindow::selectOperation(Operation op)
    {
        const auto& items = specifications();
        for (int i = 0; i < int(items.size()); ++i)
        {
            if (items[i].operation == op)
            {
                operations_->setCurrentRow(i);
            }
        }
    }

    void MainWindow::appendLog(const QString& message)
    {
        log_->appendPlainText(QDateTime::currentDateTime().toString("[HH:mm:ss] ") + message);
    }

    void MainWindow::setBusy(bool busy)
    {
        rgbdPanel_->setExternalBusy(busy);
        operationPanel_->setEnabled(!busy);
        for (auto action : changingActions_)
        {
            action->setEnabled(!busy);
        }
        runButton_->setText(busy ? "正在计算，请稍候…" : "执行算法");
        statusBar()->showMessage(busy ? "后台处理中 · 仍可旋转、平移和缩放视图" : "就绪");
    }

    void MainWindow::runTask(const QString& label,
        std::function<Result()> work,
        std::function<void(const Result&)> success)
    {
        if (isBusy())
        {
            return;
        }
        appendLog("开始：" + label);
        watcher_ = new QFutureWatcher<JobOutcome>(this);
        setBusy(true);
        auto watcher = watcher_;
        connect(watcher,
            &QFutureWatcher<JobOutcome>::finished,
            this,
            [this, watcher, success, label]
            {
                const auto outcome = watcher->result();
                watcher_ = nullptr;
                watcher->deleteLater();
                setBusy(false);
                if (!outcome.error.isEmpty())
                {
                    appendLog("失败：" + label + "\n" + outcome.error);
                    statusBar()->showMessage("操作失败，原有数据已保留。详见运行日志。");
                    return;
                }
                success(outcome.result);
                appendLog(
                    QString("完成：%1 · %2 ms").arg(label).arg(outcome.milliseconds, 0, 'f', 1));
                if (!outcome.result.detail.isEmpty())
                {
                    appendLog(outcome.result.detail);
                }
                refresh();
            });
        // 这里不捕获 this：工作线程不触碰任何 QWidget，异常转换为消息交回主线程。
        watcher->setFuture(QtConcurrent::run(
            [work]
            {
                JobOutcome outcome;
                QElapsedTimer timer;
                timer.start();
                try
                {
                    outcome.result = work();
                }
                catch (const std::exception& e)
                {
                    outcome.error = QString::fromUtf8(e.what());
                }
                catch (...)
                {
                    outcome.error = "发生未知异常。";
                }
                outcome.milliseconds = timer.nsecsElapsed() / 1000000.;
                return outcome;
            }));
    }

    void MainWindow::executeSelected()
    {
        if (isBusy())
        {
            return;
        }
        for (auto p : parameters_)
        {
            p->interpretText();
        }
        Values values{};
        const auto& spec = specification();
        for (std::size_t i = 0; i < spec.parameters.size(); ++i)
        {
            values[i] = parameters_[i]->value();
        }
        auto input = Cloud::ConstPtr(current_.cloud);
        auto target = Cloud::ConstPtr(target_);
        auto op = spec.operation;
        runTask(
            spec.title,
            [input, target, op, values]
            {
                return execute(op, input, values, target);
            },
            [this](const Result& result)
            {
                auto previousCount = current_.cloud->size();
                if (history_.size() >= 8)
                {
                    history_.erase(history_.begin());
                }
                history_.push_back(current_);
                current_ = result;
                appendLog(
                    QString("输入 %1 → 输出 %2 点").arg(previousCount).arg(current_.cloud->size()));
                // 不在每次滤波后自动重新居中，保留共同参考系，坐标变换的位移也可直接看见。
            });
    }

    void MainWindow::loadSample(int kind)
    {
        if (isBusy())
        {
            return;
        }
        target_.reset();
        history_.clear();
        current_ = Result{};
        if (kind == 2)
        {
            target_ = makeRegistrationTarget();
            original_ = displaced(target_);
            selectOperation(Operation::Icp);
        }
        else
        {
            original_ = kind == 1 ? makeSphere() : makeScene();
            selectOperation(kind == 1 ? Operation::Normal : Operation::Voxel);
        }
        current_.cloud = original_;
        if (kind == 2)
        {
            current_.target = target_;
        }
        appendLog(QString("已生成%1：%2 点。")
                .arg(kind == 2      ? " ICP 错位点云与目标"
                        : kind == 1 ? "球面示例"
                                    : "桌面场景")
                .arg(original_->size()));
        refresh();
        fitViews();
    }

    void MainWindow::openCloud(bool asTarget)
    {
        const auto filename = QFileDialog::getOpenFileName(
            this, asTarget ? "打开 ICP 目标点云" : "打开点云", {}, "点云 (*.pcd *.ply)");
        if (filename.isEmpty())
        {
            return;
        }
        runTask(
            asTarget ? "加载目标点云" : "加载点云",
            [filename]
            {
                Result r;
                r.cloud = readCloud(filename);
                r.detail = filename;
                return r;
            },
            [this, asTarget](const Result& r)
            {
                if (asTarget)
                {
                    target_ = r.cloud;
                    current_.target = target_;
                    selectOperation(Operation::Icp);
                }
                else
                {
                    original_ = r.cloud;
                    current_ = r;
                    target_.reset();
                    history_.clear();
                }
                refresh();
                fitViews();
            });
    }

    void MainWindow::saveResult(bool mesh)
    {
        if (mesh && (!current_.mesh || current_.mesh->polygons.empty()))
        {
            appendLog("尚无三角网格：先对球面示例执行“贪婪三角化”。");
            return;
        }
        QString filename = QFileDialog::getSaveFileName(this,
            mesh ? "导出三角网格" : "导出 XYZ 点云",
            mesh ? "mesh.ply" : "result.pcd",
            mesh ? "PLY 网格 (*.ply)" : "PCD 点云 (*.pcd);;PLY 点云 (*.ply)");
        if (filename.isEmpty())
        {
            return;
        }
        if (QFileInfo(filename).suffix().isEmpty())
        {
            filename += mesh ? ".ply" : ".pcd";
        }
        auto snapshot = current_;
        runTask(
            mesh ? "导出网格" : "导出点云",
            [filename, snapshot, mesh]
            {
                if (mesh)
                {
                    writeMesh(filename, *snapshot.mesh);
                }
                else
                {
                    writeCloud(filename, snapshot.cloud);
                }
                Result r;
                r.detail = "已保存：" + filename;
                return r;
            },
            [](const Result&)
            {
            });
    }

    void MainWindow::openNativeViewer()
    {
        if (!current_.cloud || current_.cloud->empty())
        {
            appendLog("无点云可显示。");
            return;
        }
        // 原生 VTK 查看器单独进程运行，避免 Qt/VTK 两套事件循环互相争用。
        QString folder = QCoreApplication::applicationDirPath() + "/output";
        QDir().mkpath(folder);
        QString filename = folder + "/native-preview-" +
            QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss-zzz") + ".pcd";
        auto cloud = Cloud::ConstPtr(current_.cloud);
        runTask(
            "准备 PCL 原生查看器",
            [cloud, filename]
            {
                writeCloud(filename, cloud);
                Result r;
                r.detail = filename;
                return r;
            },
            [this, filename](const Result&)
            {
                if (!QProcess::startDetached(
                        QCoreApplication::applicationFilePath(), {"--native-viewer", filename}))
                {
                    appendLog("无法启动 PCL 原生查看器。");
                }
                else
                {
                    appendLog("已打开独立 PCLVisualizer 窗口；预览点云保存在 output 目录。");
                }
            });
    }

    void MainWindow::undo()
    {
        if (isBusy() || history_.empty())
        {
            return;
        }
        current_ = history_.back();
        history_.pop_back();
        refresh();
        appendLog("已撤销上一步。");
    }

    void MainWindow::reset()
    {
        if (isBusy() || !original_)
        {
            return;
        }
        current_ = Result{};
        current_.cloud = original_;
        current_.target = target_;
        history_.clear();
        refresh();
        fitViews();
        appendLog("已恢复原始数据，处理链已清空。");
    }

    void MainWindow::refresh()
    {
        Result original;
        original.cloud = original_;
        originalView_->setResult(original);
        resultView_->setResult(current_);
        summary_->setText(QString("原始 %1 点   →   当前 %2 点     |     可撤销 %3 步")
                .arg(original_ ? original_->size() : 0)
                .arg(current_.cloud->size())
                .arg(history_.size()));
        targetLabel_->setText(target_
                ? QString("ICP 目标：已加载 %1 点 · 绿色叠加仅用于显示，不合并进导出结果。")
                      .arg(target_->size())
                : "ICP 目标：未加载 · 使用 ICP 示例或“加载 ICP 目标”准备数据。");
    }

    void MainWindow::fitViews()
    {
        Eigen::Vector3f low = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
        Eigen::Vector3f high = -low;
        bool any = false;
        // 两侧共享同一个包围盒，避免各自自动缩放掩盖变换、裁剪等真实差异。
        for (const auto& cloud :
            {Cloud::ConstPtr(original_), Cloud::ConstPtr(current_.cloud), Cloud::ConstPtr(target_)})
        {
            if (!cloud)
            {
                continue;
            }
            for (const auto& p : *cloud)
            {
                if (pcl::isFinite(p))
                {
                    low = low.cwiseMin(p.getVector3fMap());
                    high = high.cwiseMax(p.getVector3fMap());
                    any = true;
                }
            }
        }
        Camera camera;
        if (any)
        {
            camera.center = (low + high) * .5f;
            camera.radius = std::max(.001f, (high - low).norm() * .5f);
        }
        originalView_->setCamera(camera);
        resultView_->setCamera(camera);
    }

    void MainWindow::closeEvent(QCloseEvent* event)
    {
        if (isBusy())
        {
            event->ignore();
            statusBar()->showMessage(
                "任务尚在运行，请等待完成后关闭。PCL 运算不支持安全强制中断。");
        }
        else
        {
            event->accept();
        }
    }
} // namespace demo
