#include "PointCloudView.h"
#include <pcl/common/point_tests.h>
#include <pcl/conversions.h>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace demo
{
    PointCloudView::PointCloudView(QString title, QWidget* parent)
        : QWidget(parent), title_(std::move(title))
    {
        setMinimumSize(200, 260);
        setMouseTracking(true);
        setCursor(Qt::OpenHandCursor);
        setToolTip(
            "左键拖动：旋转\n右键 / 中键拖动：平移\n滚轮：缩放\n双击：恢复观察角度\n两侧相机同步");
    }

    void PointCloudView::setResult(const Result& data)
    {
        data_ = data;
        update();
    }

    void PointCloudView::setCamera(const Camera& camera)
    {
        camera_ = camera;
        update();
    }

    QPointF PointCloudView::project(const Eigen::Vector3f& point, double* depth) const
    {
        const auto d = point - camera_.center;
        double x = std::cos(camera_.yaw) * d.x() + std::sin(camera_.yaw) * d.y();
        double y = -std::sin(camera_.yaw) * d.x() + std::cos(camera_.yaw) * d.y();
        double z = -std::sin(camera_.pitch) * y + std::cos(camera_.pitch) * d.z();
        if (depth)
        {
            *depth = std::cos(camera_.pitch) * y + std::sin(camera_.pitch) * d.z();
        }
        double scale =
            .40 * std::min(width(), height()) * camera_.zoom / std::max(.0001f, camera_.radius);
        return {width() / 2. + x * scale + camera_.pan.x(),
            height() / 2. - z * scale + camera_.pan.y()};
    }

    void PointCloudView::paintEvent(QPaintEvent*)
    {
        QPainter painter(this);
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
        if (showMesh_ && data_.mesh && !data_.mesh->polygons.empty())
        {
            Cloud vertices;
            pcl::fromPCLPointCloud2(data_.mesh->cloud, vertices);
            painter.setPen(QPen(QColor(100, 201, 240, 125), .7));
            // 教学查看器以线框显示三角面，导出的 PLY 仍包含全部网格拓扑。
            std::size_t step =
                std::max<std::size_t>(1, (data_.mesh->polygons.size() + 11999) / 12000);
            for (std::size_t i = 0; i < data_.mesh->polygons.size(); i += step)
            {
                const auto& poly = data_.mesh->polygons[i].vertices;
                if (poly.size() != 3)
                {
                    continue;
                }
                QPolygonF triangle;
                for (auto index : poly)
                {
                    if (index < vertices.size())
                    {
                        triangle << project(vertices[index].getVector3fMap());
                    }
                }
                if (triangle.size() == 3)
                {
                    painter.drawPolygon(triangle);
                }
            }
        }

        struct Dot
        {
            QPointF xy;
            double depth;
            Color color;
        };

        std::vector<Dot> dots;
        auto append = [&](const Cloud::ConstPtr& cloud, bool target)
        {
            if (!cloud)
            {
                return;
            }
            auto stride = std::max<std::size_t>(1, (cloud->size() + 44999) / 45000);
            for (std::size_t i = 0; i < cloud->size(); i += stride)
            {
                const auto& point = (*cloud)[i];
                if (!pcl::isFinite(point))
                {
                    continue;
                }
                Dot dot;
                dot.xy = project(point.getVector3fMap(), &dot.depth);
                dot.color = target
                    ? Color{68, 223, 153}
                    : (data_.colors.size() == cloud->size() ? data_.colors[i] : Color{});
                dots.push_back(dot);
            }
        };
        append(data_.cloud, false);
        append(data_.target, true);
        // 远处先画、近处后画，保证覆盖关系；不改变原始点云索引。
        std::sort(dots.begin(),
            dots.end(),
            [](const Dot& a, const Dot& b)
            {
                return a.depth < b.depth;
            });
        painter.setRenderHint(QPainter::Antialiasing, false);
        QPen pen;
        pen.setWidth(pointSize_);
        pen.setCapStyle(Qt::RoundCap);
        for (const auto& dot : dots)
        {
            pen.setColor(QColor(dot.color.r, dot.color.g, dot.color.b));
            painter.setPen(pen);
            painter.drawPoint(dot.xy);
        }
        painter.setRenderHint(QPainter::Antialiasing);
        if (showNormals_ && data_.normals && data_.normals->size() == data_.cloud->size())
        {
            painter.setPen(QPen(QColor(255, 204, 110, 175), 1));
            auto step = std::max<std::size_t>(1, (data_.cloud->size() + 449) / 450);
            for (std::size_t i = 0; i < data_.cloud->size(); i += step)
            {
                const auto& n = (*data_.normals)[i];
                Eigen::Vector3f normal(n.normal_x, n.normal_y, n.normal_z);
                if (!normal.allFinite())
                {
                    continue;
                }
                Eigen::Vector3f start = (*data_.cloud)[i].getVector3fMap();
                painter.drawLine(
                    project(start), project(start + normal * float(data_.normalScale)));
            }
        }
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
        update();
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
        update();
        emit cameraChanged();
        event->accept();
    }

    void PointCloudView::mouseDoubleClickEvent(QMouseEvent*)
    {
        camera_.yaw = .6;
        camera_.pitch = .5;
        camera_.zoom = 1;
        camera_.pan = {};
        update();
        emit cameraChanged();
    }
} // namespace demo
