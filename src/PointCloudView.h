#pragma once
#include "Algorithms.h"
#include <QWidget>
#include <QPointF>
#include <QImage>
#include <QTimer>

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
    // 静止时每组最多显示 45,000 点，交互期间降低显示密度；处理和导出保留完整点云。
    class PointCloudView : public QWidget
    {
        Q_OBJECT

      public:
        explicit PointCloudView(QString title, QWidget* parent = nullptr);
        void setResult(const Result& data);
        void setCamera(const Camera& camera, bool interacting = false);

        const Camera& camera() const
        {
            return camera_;
        }

        void setPointSize(int size)
        {
            pointSize_ = size;
            invalidateFrame();
        }

        void setShowNormals(bool enabled)
        {
            showNormals_ = enabled;
            invalidateFrame();
        }

        void setShowMesh(bool enabled)
        {
            showMesh_ = enabled;
            invalidateFrame();
        }

      signals:
        void cameraChanged();

      protected:
        void changeEvent(QEvent* event) override;
        void paintEvent(QPaintEvent*) override;
        void mousePressEvent(QMouseEvent*) override;
        void mouseMoveEvent(QMouseEvent*) override;
        void mouseReleaseEvent(QMouseEvent*) override;
        void wheelEvent(QWheelEvent*) override;
        void mouseDoubleClickEvent(QMouseEvent*) override;

      private:
        struct DisplayPoint
        {
            Eigen::Vector3f position;
            QRgb color;
        };

        void invalidateFrame();
        void beginInteraction();
        void rebuildDisplayData();
        void renderFrame();
        void renderPoints();
        QPointF project(const Eigen::Vector3f& point, double* depth = nullptr) const;
        QString title_;
        Result data_;
        Camera camera_;
        QPoint previousMouse_;
        int pointSize_ = 3;
        bool showNormals_ = true;
        bool showMesh_ = true;
        std::vector<DisplayPoint> cloudPoints_;
        std::vector<DisplayPoint> targetPoints_;
        std::vector<std::array<Eigen::Vector3f, 3>> triangles_;
        std::vector<std::array<Eigen::Vector3f, 2>> normalLines_;
        QTimer interactionTimer_;
        QImage frame_;
        QImage pointLayer_;
        std::vector<double> depthBuffer_;
        bool frameDirty_ = true;
        Eigen::Vector3d screenX_;
        Eigen::Vector3d screenY_;
        Eigen::Vector3d depthDirection_;
        QPointF screenCenter_;
    };
} // namespace demo
