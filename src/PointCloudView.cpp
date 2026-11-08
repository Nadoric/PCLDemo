#include "PointCloudView.h"
#include <pcl/common/point_tests.h>
#include <pcl/conversions.h>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <limits>

namespace demo
{
    PointCloudView::PointCloudView(QString title, QWidget* parent)
        : QWidget(parent), title_(std::move(title))
    {
        setMinimumSize(200, 260);
        setAttribute(Qt::WA_OpaquePaintEvent);
        interactionTimer_.setSingleShot(true);
        interactionTimer_.setInterval(140);
        connect(&interactionTimer_, &QTimer::timeout, this, &PointCloudView::invalidateFrame);
        setMouseTracking(true);
        setCursor(Qt::OpenHandCursor);
        setToolTip(
            "左键拖动：旋转\n右键 / 中键拖动：平移\n滚轮：缩放\n双击：恢复观察角度\n两侧相机同步");
    }

    void PointCloudView::setResult(const Result& data)
    {
        data_ = data;
        rebuildDisplayData();
        invalidateFrame();
    }

    void PointCloudView::setCamera(const Camera& camera, bool interacting)
    {
        camera_ = camera;
        if (interacting)
        {
            beginInteraction();
        }
        else
        {
            interactionTimer_.stop();
            invalidateFrame();
        }
    }

    void PointCloudView::invalidateFrame()
    {
        frameDirty_ = true;
        update();
    }

    void PointCloudView::beginInteraction()
    {
        interactionTimer_.start();
        invalidateFrame();
    }

    void PointCloudView::rebuildDisplayData()
    {
        auto sample =
            [&](const Cloud::ConstPtr& cloud, std::vector<DisplayPoint>& points, bool target)
        {
            points.clear();
            if (!cloud)
            {
                return;
            }
            points.reserve(std::min<std::size_t>(cloud->size(), 45000));
            const auto stride = std::max<std::size_t>(1, (cloud->size() + 44999) / 45000);
            for (std::size_t i = 0; i < cloud->size(); i += stride)
            {
                const auto& point = (*cloud)[i];
                if (!pcl::isFinite(point))
                {
                    continue;
                }
                const Color color = target
                    ? Color{68, 223, 153}
                    : (data_.colors.size() == cloud->size() ? data_.colors[i] : Color{});
                points.push_back({point.getVector3fMap(), qRgb(color.r, color.g, color.b)});
            }
        };
        sample(data_.cloud, cloudPoints_, false);
        sample(data_.target, targetPoints_, true);

        // 网格格式转换和有限性检查仅在结果变化时执行，不进入逐帧绘制路径。
        triangles_.clear();
        if (data_.mesh && !data_.mesh->polygons.empty())
        {
            Cloud vertices;
            pcl::fromPCLPointCloud2(data_.mesh->cloud, vertices);
            const auto step =
                std::max<std::size_t>(1, (data_.mesh->polygons.size() + 11999) / 12000);
            for (std::size_t i = 0; i < data_.mesh->polygons.size(); i += step)
            {
                const auto& indices = data_.mesh->polygons[i].vertices;
                if (indices.size() != 3)
                {
                    continue;
                }
                std::array<Eigen::Vector3f, 3> triangle;
                bool valid = true;
                for (std::size_t j = 0; j < 3; ++j)
                {
                    if (indices[j] < 0 || std::size_t(indices[j]) >= vertices.size() ||
                        !pcl::isFinite(vertices[indices[j]]))
                    {
                        valid = false;
                        break;
                    }
                    triangle[j] = vertices[indices[j]].getVector3fMap();
                }
                if (valid)
                {
                    triangles_.push_back(triangle);
                }
            }
        }

        normalLines_.clear();
        if (data_.cloud && data_.normals && data_.normals->size() == data_.cloud->size())
        {
            const auto step = std::max<std::size_t>(1, (data_.cloud->size() + 449) / 450);
            for (std::size_t i = 0; i < data_.cloud->size(); i += step)
            {
                const auto& normal = (*data_.normals)[i];
                const Eigen::Vector3f direction(normal.normal_x, normal.normal_y, normal.normal_z);
                const Eigen::Vector3f start = (*data_.cloud)[i].getVector3fMap();
                const Eigen::Vector3f end = start + direction * float(data_.normalScale);
                if (start.allFinite() && end.allFinite())
                {
                    normalLines_.push_back({start, end});
                }
            }
        }
    }

    QPointF PointCloudView::project(const Eigen::Vector3f& point, double* depth) const
    {
        // 避免在每个点上构造 Eigen 表达式，Debug 构建也保持轻量。
        const auto* position = point.data();
        const auto* center = camera_.center.data();
        const double x = double(position[0]) - center[0];
        const double y = double(position[1]) - center[1];
        const double z = double(position[2]) - center[2];
        const auto* horizontal = screenX_.data();
        const auto* vertical = screenY_.data();
        if (depth)
        {
            const auto* direction = depthDirection_.data();
            *depth = direction[0] * x + direction[1] * y + direction[2] * z;
        }
        return screenCenter_ +
            QPointF(horizontal[0] * x + horizontal[1] * y + horizontal[2] * z,
                vertical[0] * x + vertical[1] * y + vertical[2] * z);
    }

    void PointCloudView::paintEvent(QPaintEvent*)
    {
        const qreal ratio = devicePixelRatioF();
        const QSize pixels(qCeil(width() * ratio), qCeil(height() * ratio));
        if (frame_.size() != pixels || frame_.devicePixelRatio() != ratio)
        {
            frame_ = QImage(pixels, QImage::Format_ARGB32_Premultiplied);
            frame_.setDevicePixelRatio(ratio);
            frameDirty_ = true;
        }
        if (frameDirty_)
        {
            renderFrame();
            frameDirty_ = false;
        }
        QPainter painter(this);
        painter.drawImage(QPointF(), frame_);
    }

    void PointCloudView::changeEvent(QEvent* event)
    {
        QWidget::changeEvent(event);
        invalidateFrame();
    }

    void PointCloudView::renderFrame()
    {
        // 每帧只计算一次相机矩阵，所有点、网格、法线共用同一投影。
        const double cy = std::cos(camera_.yaw);
        const double sy = std::sin(camera_.yaw);
        const double cp = std::cos(camera_.pitch);
        const double sp = std::sin(camera_.pitch);
        const double scale =
            .40 * std::min(width(), height()) * camera_.zoom / std::max(.0001f, camera_.radius);
        screenX_ = Eigen::Vector3d(cy, sy, 0.) * scale;
        screenY_ = Eigen::Vector3d(-sp * sy, sp * cy, -cp) * scale;
        depthDirection_ = Eigen::Vector3d(-cp * sy, cp * cy, sp);
        screenCenter_ = QPointF(width() / 2., height() / 2.) + camera_.pan;

        QPainter painter(&frame_);
        painter.setFont(font());
        painter.fillRect(rect(), QColor("#101c2c"));
        painter.setRenderHint(QPainter::Antialiasing);
        // 地面网格与坐标轴使用相同投影，帮助理解三维方向与坐标变换。
        float r = camera_.radius;
        const auto c = camera_.center;
        painter.setPen(QPen(QColor("#23344b"), 1));
        for (int i = -5; i <= 5; ++i)
        {
            float v = i * r / 5;
            painter.drawLine(project(c + Eigen::Vector3f(-r, v, -r * .55f)),
                project(c + Eigen::Vector3f(r, v, -r * .55f)));
            painter.drawLine(project(c + Eigen::Vector3f(v, -r, -r * .55f)),
                project(c + Eigen::Vector3f(v, r, -r * .55f)));
        }
        const bool interacting = interactionTimer_.isActive();
        if (showMesh_)
        {
            painter.setRenderHint(QPainter::Antialiasing, !interacting);
            painter.setPen(QPen(QColor(100, 201, 240, 125), .7));
            const auto step =
                interacting ? std::max<std::size_t>(1, (triangles_.size() + 1999) / 2000) : 1;
            for (std::size_t i = 0; i < triangles_.size(); i += step)
            {
                const auto& triangle = triangles_[i];
                const QPointF points[] = {
                    project(triangle[0]), project(triangle[1]), project(triangle[2])};
                painter.drawPolygon(points, 3);
            }
        }

        renderPoints();
        painter.drawImage(QPointF(), pointLayer_);
        painter.setRenderHint(QPainter::Antialiasing, !interacting);
        if (showNormals_)
        {
            painter.setPen(QPen(QColor(255, 204, 110, 175), 1));
            const std::size_t step = interacting ? 3 : 1;
            for (std::size_t i = 0; i < normalLines_.size(); i += step)
            {
                painter.drawLine(project(normalLines_[i][0]), project(normalLines_[i][1]));
            }
        }
        painter.setRenderHint(QPainter::Antialiasing);
        // 小型坐标三叉在视口左下角，不受平移影响。
        QPointF anchor(38, height() - 42);
        auto zero = project(camera_.center);
        const Eigen::Vector3f axes[] = {
            Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitZ()};
        const QColor axisColors[] = {QColor("#f98389"), QColor("#67dca9"), QColor("#77adff")};
        for (int i = 0; i < 3; ++i)
        {
            QPointF delta = project(camera_.center + axes[i] * camera_.radius) - zero;
            if (std::hypot(delta.x(), delta.y()) > 0)
            {
                delta *= 25. / std::max(1., std::hypot(delta.x(), delta.y()));
            }
            painter.setPen(QPen(axisColors[i], 2));
            painter.drawLine(anchor, anchor + delta);
            painter.drawText(anchor + delta + QPointF(3, -2), QString(QChar('X' + i)));
        }
        painter.setPen(QColor("#e2edf7"));
        QFont labelFont = font();
        labelFont.setBold(true);
        labelFont.setPointSize(11);
        painter.setFont(labelFont);
        painter.drawText(QRect(18, 14, width() - 36, 26), title_);
        labelFont.setBold(false);
        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.setPen(QColor("#97b0ca"));
        QString stats = QString("%1 点").arg(data_.cloud ? data_.cloud->size() : 0);
        if (data_.cloud && data_.cloud->size() > 45000)
        {
            stats += " · 显示已抽样";
        }
        if (data_.mesh)
        {
            stats += QString(" · %1 面 / 线框").arg(data_.mesh->polygons.size());
        }
        if (data_.target)
        {
            stats += " · 绿色：目标";
        }
        painter.drawText(QRect(18, 43, width() - 36, 24), stats);
        if (!data_.cloud || data_.cloud->empty())
        {
            painter.setPen(QColor("#9eb3ca"));
            painter.drawText(rect(), Qt::AlignCenter, "暂无点云\n加载文件或生成示例；空结果可撤销");
        }
        painter.setPen(QColor("#6f879f"));
        painter.drawText(QRect(90, height() - 29, width() - 105, 20),
            Qt::AlignRight,
            "拖动旋转 · 右键平移 · 滚轮缩放");
    }

    void PointCloudView::renderPoints()
    {
        const qreal ratio = frame_.devicePixelRatio();
        if (pointLayer_.size() != frame_.size())
        {
            pointLayer_ = QImage(frame_.size(), QImage::Format_ARGB32_Premultiplied);
        }
        pointLayer_.setDevicePixelRatio(ratio);
        pointLayer_.fill(Qt::transparent);
        const int pixelWidth = pointLayer_.width();
        const int pixelHeight = pointLayer_.height();
        auto* pixels = reinterpret_cast<QRgb*>(pointLayer_.bits());
        const int rowStride = pointLayer_.bytesPerLine() / int(sizeof(QRgb));
        depthBuffer_.resize(std::size_t(pixelWidth) * pixelHeight);
        std::fill(
            depthBuffer_.begin(), depthBuffer_.end(), -std::numeric_limits<double>::infinity());
        auto* depths = depthBuffer_.data();

        // 点精灵直接写入像素深度缓冲：无需排序，也无需逐点切换 QPainter 画笔。
        // 所有缓冲按实际设备像素分配，保证高 DPI 下的点大小与遮挡关系一致。
        const double radius = std::max(.5, pointSize_ * ratio / 2.);
        const int extent = int(std::ceil(radius));
        std::vector<QPoint> footprint;
        for (int y = -extent; y <= extent; ++y)
        {
            for (int x = -extent; x <= extent; ++x)
            {
                if (x * x + y * y <= radius * radius)
                {
                    footprint.emplace_back(x, y);
                }
            }
        }
        auto draw = [&](const std::vector<DisplayPoint>& points)
        {
            const auto step = interactionTimer_.isActive()
                ? std::max<std::size_t>(1, (points.size() + 11999) / 12000)
                : 1;
            for (std::size_t i = 0; i < points.size(); i += step)
            {
                const auto& point = points.data()[i];
                double depth = 0.;
                const QPointF xy = project(point.position, &depth) * ratio;
                if (!std::isfinite(xy.x()) || !std::isfinite(xy.y()) || !std::isfinite(depth) ||
                    xy.x() < -extent || xy.y() < -extent || xy.x() >= pixelWidth + extent ||
                    xy.y() >= pixelHeight + extent)
                {
                    continue;
                }
                const int centerX = qRound(xy.x());
                const int centerY = qRound(xy.y());
                for (std::size_t j = 0; j < footprint.size(); ++j)
                {
                    const auto& offset = footprint.data()[j];
                    const int x = centerX + offset.x();
                    const int y = centerY + offset.y();
                    if (x < 0 || x >= pixelWidth || y < 0 || y >= pixelHeight)
                    {
                        continue;
                    }
                    const auto index = std::size_t(y) * pixelWidth + x;
                    if (depth >= depths[index])
                    {
                        depths[index] = depth;
                        pixels[std::size_t(y) * rowStride + x] = point.color;
                    }
                }
            }
        };
        draw(cloudPoints_);
        draw(targetPoints_);
    }

    void PointCloudView::mousePressEvent(QMouseEvent* event)
    {
        previousMouse_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
    }

    void PointCloudView::mouseMoveEvent(QMouseEvent* event)
    {
        const auto delta = event->pos() - previousMouse_;
        previousMouse_ = event->pos();
        if (event->buttons() & Qt::LeftButton)
        {
            camera_.yaw += delta.x() * .008;
            camera_.pitch = std::clamp(camera_.pitch + delta.y() * .008, -1.5, 1.5);
        }
        else if (event->buttons() & (Qt::MiddleButton | Qt::RightButton))
        {
            camera_.pan += delta;
        }
        else
        {
            return;
        }
        beginInteraction();
        emit cameraChanged();
    }

    void PointCloudView::mouseReleaseEvent(QMouseEvent*)
    {
        setCursor(Qt::OpenHandCursor);
    }

    void PointCloudView::wheelEvent(QWheelEvent* event)
    {
        camera_.zoom =
            std::clamp(camera_.zoom * std::pow(1.0015, event->angleDelta().y()), .05, 100.);
        beginInteraction();
        emit cameraChanged();
        event->accept();
    }

    void PointCloudView::mouseDoubleClickEvent(QMouseEvent*)
    {
        camera_.yaw = .6;
        camera_.pitch = .5;
        camera_.zoom = 1;
        camera_.pan = {};
        beginInteraction();
        emit cameraChanged();
    }
} // namespace demo
