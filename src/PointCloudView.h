#pragma once
#include "Algorithms.h"
#include <QWidget>
#include <QPointF>

namespace demo
{
    struct Camera
    {
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        float radius = 2.5f;
        double yaw = .6;
        double pitch = .5;
        double zoom = 1.;
        QPointF pan;
    };

    // 嵌入式教学查看器由 Qt 绘制，不依赖安装包未提供的 QVTK Qt 插件。
    // 它只负责显示：滤波、搜索、分割、重建等计算全部由 Algorithms.cpp 的 PCL 完成。
    // 为保证大点云操作流畅，显示抽样最多 45,000 点；处理和导出仍使用完整点云。
    class PointCloudView : public QWidget
    {
        Q_OBJECT

      public:
        explicit PointCloudView(QString title, QWidget* parent = nullptr);
        void setResult(const Result& data);
        void setCamera(const Camera& camera);

        const Camera& camera() const
        {
            return camera_;
        }

        void setPointSize(int size)
        {
            pointSize_ = size;
            update();
        }

        void setShowNormals(bool enabled)
        {
            showNormals_ = enabled;
            update();
        }

        void setShowMesh(bool enabled)
        {
            showMesh_ = enabled;
            update();
        }

      signals:
        void cameraChanged();

      protected:
        void paintEvent(QPaintEvent*) override;
        void mousePressEvent(QMouseEvent*) override;
        void mouseMoveEvent(QMouseEvent*) override;
        void mouseReleaseEvent(QMouseEvent*) override;
        void wheelEvent(QWheelEvent*) override;
        void mouseDoubleClickEvent(QMouseEvent*) override;

      private:
        QPointF project(const Eigen::Vector3f& point, double* depth = nullptr) const;
        QString title_;
        Result data_;
        Camera camera_;
        QPoint previousMouse_;
        int pointSize_ = 3;
        bool showNormals_ = true;
        bool showMesh_ = true;
    };
} // namespace demo
