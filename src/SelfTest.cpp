#include "SelfTest.h"
#include "Algorithms.h"
#include "MainWindow.h"
#include "PointCloudView.h"
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <QApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QFontMetrics>
#include <QListWidget>
#include <QMouseEvent>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSaveFile>
#include <QTemporaryDir>
#include <QThread>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>

namespace demo
{
    namespace
    {
        Values defaults(Operation op)
        {
            Values values{};
            for (const auto& spec : specifications())
            {
                if (spec.operation == op)
                {
                    for (std::size_t i = 0; i < spec.parameters.size(); ++i)
                    {
                        values[i] = spec.parameters[i].value;
                    }
                }
            }
            return values;
        }

        void check(bool value, const char* message)
        {
            if (!value)
            {
                throw std::runtime_error(message);
            }
        }

        void waitForWindow(MainWindow& window)
        {
            QElapsedTimer timer;
            timer.start();
            while (window.isBusy() && timer.elapsed() < 60000)
            {
                QApplication::processEvents(QEventLoop::AllEvents, 20);
                QThread::msleep(5);
            }
            check(!window.isBusy(), "UI worker did not complete before timeout");
            QApplication::processEvents();
        }
    } // namespace

    int runSelfTest(const QString& reportPath)
    {
        QStringList report;
        int failures = 0;
        int passes = 0;
        auto test = [&](QString name, const std::function<void()>& function)
        {
            try
            {
                function();
                report << "PASS  " + name;
                ++passes;
            }
            catch (const std::exception& e)
            {
                report << "FAIL  " + name + " : " + QString::fromUtf8(e.what());
                ++failures;
            }
        };
        auto scene = makeScene();
        auto sphere = makeSphere();
        test("3D preview: dense colored cloud / target / mesh rendering",
            [&]
            {
                Result dense;
                for (int i = 0; i < 45000; ++i)
                {
                    float x = float(i % 300) / 150.f - 1.f;
                    float y = float(i / 300) / 75.f - 1.f;
                    dense.cloud->push_back(Point(x, y, .2f * std::sin(x * 5.f)));
                    dense.colors.push_back(Color{static_cast<unsigned char>(i % 256), 189, 227});
                }
                dense.target = dense.cloud;
                dense.mesh.reset(new pcl::PolygonMesh);
                pcl::toPCLPointCloud2(*dense.cloud, dense.mesh->cloud);
                for (int i = 0; i < 12000; ++i)
                {
                    pcl::Vertices triangle;
                    triangle.vertices = {i, i + 1, i + 300};
                    dense.mesh->polygons.push_back(std::move(triangle));
                }

                PointCloudView view("Dense preview");
                view.resize(800, 600);
                view.setResult(dense);
                QImage frame(view.size(), QImage::Format_ARGB32_Premultiplied);
                view.render(&frame);
                const QImage initial = frame.copy();
                QElapsedTimer timer;
                timer.start();
                for (int i = 0; i < 20; ++i)
                {
                    Camera camera = view.camera();
                    camera.yaw += .025;
                    view.setCamera(camera);
                    view.render(&frame);
                }
                const double movingMs = timer.nsecsElapsed() / 1.e6 / 20.;
                check(frame != initial, "camera rotation did not change preview");
                const QImage stationary = frame.copy();
                timer.restart();
                for (int i = 0; i < 20; ++i)
                {
                    view.render(&frame);
                }
                const double stationaryMs = timer.nsecsElapsed() / 1.e6 / 20.;
                check(frame == stationary, "unchanged preview is not stable");
                view.setCamera(view.camera(), true);
                view.render(&frame);
                check(frame != stationary, "interaction did not reduce preview density");
                timer.restart();
                while (timer.elapsed() < 180)
                {
                    QApplication::processEvents();
                    QThread::msleep(5);
                }
                view.render(&frame);
                check(frame == stationary, "preview detail did not recover after interaction");
                timer.restart();
                for (int i = 0; i < 20; ++i)
                {
                    Camera camera = view.camera();
                    camera.yaw += .025;
                    view.setCamera(camera, true);
                    view.render(&frame);
                }
                const double interactingMs = timer.nsecsElapsed() / 1.e6 / 20.;
                view.setShowMesh(false);
                view.setPointSize(6);
                view.render(&frame);
                check(frame != stationary, "display controls did not update preview");
                view.setResult(Result{});
                view.render(&frame);
                check(frame != stationary, "empty result retained old points");
                check(dense.cloud->size() == 45000 && dense.mesh->polygons.size() == 12000,
                    "preview modified processing data");
                report << QString("PERF  800x600 / 90000 points / 12000 triangles: "
                                  "rotating %1 ms/frame; stationary %2 ms/frame")
                              .arg(movingMs, 0, 'f', 2)
                              .arg(stationaryMs, 0, 'f', 2);
                report << QString("PERF  interactive preview: %1 ms/frame")
                              .arg(interactingMs, 0, 'f', 2);
            });
        test("3D preview: depth occlusion / clipping / invalid data / resize",
            [&]
            {
                Result points;
                points.cloud->push_back(Point(0.f, 1.f, 0.f));
                points.cloud->push_back(Point(0.f, -1.f, 0.f));
                points.cloud->push_back(Point(1.e20f, 0.f, 0.f));
                points.cloud->push_back(Point(std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f));
                points.colors = {Color{30, 90, 250}, Color{250, 40, 30}, Color{}, Color{}};
                Cloud::Ptr target(new Cloud);
                target->push_back(Point(0.f, -2.f, 0.f));
                points.target = target;
                points.mesh.reset(new pcl::PolygonMesh);
                pcl::toPCLPointCloud2(*points.cloud, points.mesh->cloud);
                pcl::Vertices invalid;
                invalid.vertices = {-1, 0, 999};
                points.mesh->polygons.push_back(invalid);

                PointCloudView view("Depth preview");
                view.resize(400, 300);
                Camera camera;
                camera.yaw = 0.;
                camera.pitch = 0.;
                camera.radius = 1.f;
                view.setCamera(camera);
                view.setResult(points);
                QImage frame(view.size(), QImage::Format_ARGB32_Premultiplied);
                view.render(&frame);
                check(frame.pixelColor(200, 150) == QColor(30, 90, 250),
                    "farther cloud or target overwrote nearer point");

                target->front().y = 2.f;
                view.setResult(points);
                view.render(&frame);
                check(frame.pixelColor(200, 150) == QColor(68, 223, 153),
                    "new result did not invalidate cached target or depth buffer");
                view.resize(640, 480);
                frame = QImage(view.size(), QImage::Format_ARGB32_Premultiplied);
                view.render(&frame);
                check(frame.pixelColor(320, 240) == QColor(68, 223, 153),
                    "resize did not rebuild projection or pixel buffers");
                camera.pan = QPointF(40., 30.);
                view.setCamera(camera);
                view.render(&frame);
                check(frame.pixelColor(360, 270) == QColor(68, 223, 153),
                    "pan did not update projection");
            });
        test("UI font contains Latin and Chinese glyphs",
            [&]
            {
                QFontMetrics metrics(QApplication::font());
                check(metrics.inFontUcs4('A') && metrics.inFontUcs4(0x70b9), "font glyphs missing");
            });
        test("deterministic samples / finite coordinates",
            [&]
            {
                check(scene->size() > 3500 && sphere->size() == 3600, "sample sizes");
                auto again = makeScene();
                for (std::size_t i = 0; i < scene->size(); ++i)
                {
                    check(pcl::isFinite((*scene)[i]) && (*scene)[i].x == (*again)[i].x,
                        "sample reproducibility");
                }
            });
        test("VoxelGrid reduces points without modifying input",
            [&]
            {
                auto count = scene->size();
                auto r = execute(Operation::Voxel, scene, defaults(Operation::Voxel));
                check(!r.cloud->empty() && r.cloud->size() < count && scene->size() == count,
                    "voxel result");
            });
        test("PassThrough respects Z bounds",
            [&]
            {
                auto r = execute(Operation::PassThrough, scene, defaults(Operation::PassThrough));
                check(!r.cloud->empty(), "empty pass-through");
                for (const auto& p : *r.cloud)
                {
                    check(p.z >= .1f && p.z <= 1.5f, "Z outside bounds");
                }
            });
        for (auto op : {Operation::Statistical, Operation::RadiusOutlier})
        {
            test(op == Operation::Statistical ? "SOR removes scattered noise"
                                              : "RadiusOutlier removes sparse points",
                [&, op]
                {
                    auto r = execute(op, scene, defaults(op));
                    check(r.cloud->size() > 2000 && r.cloud->size() < scene->size(),
                        "outlier removal count");
                });
        }
        Cloud::Ptr segmented;
        test("RANSAC + ExtractIndices removes dominant plane",
            [&]
            {
                auto r = execute(Operation::Plane, scene, defaults(Operation::Plane));
                segmented = r.cloud;
                check(segmented->size() > 2000 && segmented->size() < 2500,
                    "dominant plane inlier count");
            });
        test("Euclidean clustering identifies three separated objects",
            [&]
            {
                check(bool(segmented), "segmentation prerequisite");
                auto r = execute(Operation::Clusters, segmented, defaults(Operation::Clusters));
                std::set<int> palette;
                for (auto c : r.colors)
                {
                    palette.insert((int(c.r) << 16) | (int(c.g) << 8) | c.b);
                }
                check(palette.size() == 3 && r.cloud->size() >= 2160, "cluster count / membership");
                check(r.colors.size() == r.cloud->size(), "cluster coloring");
            });
        test("NormalEstimation produces finite unit normals",
            [&]
            {
                auto r = execute(Operation::Normal, sphere, defaults(Operation::Normal));
                check(r.normals && r.normals->size() == sphere->size(), "normal output count");
                for (const auto& n : *r.normals)
                {
                    float norm = std::sqrt(n.normal_x * n.normal_x + n.normal_y * n.normal_y +
                        n.normal_z * n.normal_z);
                    check(std::isfinite(norm) && std::abs(norm - 1) < .001f, "invalid normal");
                }
            });
        test("FPFH computes 33-dimensional descriptors",
            [&]
            {
                auto r = execute(Operation::Fpfh, sphere, defaults(Operation::Fpfh));
                check(r.colors.size() == sphere->size() && r.detail.contains("3600 / 3600"),
                    "FPFH finite descriptors");
            });
        test("MLS smooths noisy sphere while retaining surface",
            [&]
            {
                auto r = execute(Operation::Mls, sphere, defaults(Operation::Mls));
                check(r.cloud->size() > 3000 && r.normals && r.normals->size() == r.cloud->size(),
                    "MLS output");
                double before = 0;
                double after = 0;
                for (const auto& p : *sphere)
                {
                    before += std::pow(p.getVector3fMap().norm() - 1, 2);
                }
                for (const auto& p : *r.cloud)
                {
                    check(pcl::isFinite(p), "MLS finite points");
                    after += std::pow(p.getVector3fMap().norm() - 1, 2);
                }
                check(after / r.cloud->size() < before / sphere->size(),
                    "MLS should reduce radial noise");
            });
        pcl::PolygonMesh::Ptr mesh;
        test("GP3 builds valid triangle topology",
            [&]
            {
                auto r = execute(Operation::Triangulate, sphere, defaults(Operation::Triangulate));
                mesh = r.mesh;
                check(mesh && mesh->polygons.size() > 1000, "mesh triangles");
                for (const auto& polygon : mesh->polygons)
                {
                    check(polygon.vertices.size() == 3, "non-triangle face");
                    for (auto index : polygon.vertices)
                    {
                        check(index < sphere->size(), "mesh index");
                    }
                }
            });
        test("KdTree K nearest returns K highlighted points",
            [&]
            {
                auto r = execute(Operation::KNearest, sphere, defaults(Operation::KNearest));
                int highlighted = 0;
                for (auto c : r.colors)
                {
                    if (c.r == 255)
                    {
                        ++highlighted;
                    }
                }
                check(highlighted == 40, "nearest K highlighted points");
            });
        test("KdTree radius returns only points within radius",
            [&]
            {
                auto values = defaults(Operation::RadiusSearch);
                auto r = execute(Operation::RadiusSearch, sphere, values);
                int found = 0;
                for (std::size_t i = 0; i < r.colors.size(); ++i)
                {
                    if (r.colors[i].r == 255)
                    {
                        check(((*sphere)[i].getVector3fMap() - (*sphere)[0].getVector3fMap())
                                    .norm() <= float(values[1]) + .00001f,
                            "radius search bounds");
                        ++found;
                    }
                }
                check(found > 1, "radius search found no neighbors");
            });
        test("Octree returns occupied voxel centers",
            [&]
            {
                auto r = execute(Operation::Octree, scene, defaults(Operation::Octree));
                check(!r.cloud->empty() && r.cloud->size() < scene->size(), "occupied voxel count");
            });
        test("Eigen / transformPointCloud uses homogeneous coordinates",
            [&]
            {
                auto r = execute(Operation::Transform, scene, defaults(Operation::Transform));
                auto expected = r.transform * (*scene)[100].getVector4fMap();
                check((expected.head<3>() - (*r.cloud)[100].getVector3fMap()).norm() < .00001f,
                    "transform mismatch");
            });
        test("ICP recovers known displacement",
            [&]
            {
                auto target = makeRegistrationTarget();
                auto source = displaced(target);
                auto r = execute(Operation::Icp, source, defaults(Operation::Icp), target);
                check(r.converged && r.fitness < .00001, "ICP convergence / fitness");
                double error = 0;
                for (std::size_t i = 0; i < target->size(); ++i)
                {
                    error += ((*r.cloud)[i].getVector3fMap() - (*target)[i].getVector3fMap())
                                 .squaredNorm();
                }
                check(error / target->size() < .00001, "ICP correspondence error");
            });
        QTemporaryDir temp(QCoreApplication::applicationDirPath() + "/test-XXXXXX");
        test("Unicode PCD / PLY binary export and reload round trip",
            [&]
            {
                check(temp.isValid(), "temporary test directory");
                for (QString suffix : {"pcd", "ply"})
                {
                    QString filename = temp.filePath("中文点云." + suffix);
                    writeCloud(filename, scene);
                    auto read = readCloud(filename);
                    check(read->size() == scene->size(), "IO count mismatch");
                    for (std::size_t i = 0; i < read->size(); ++i)
                    {
                        check(((*read)[i].getVector3fMap() - (*scene)[i].getVector3fMap()).norm() <
                                .00001f,
                            "IO point mismatch");
                    }
                }
            });
        test("Triangle mesh PLY export",
            [&]
            {
                check(bool(mesh), "mesh prerequisite");
                QString path = temp.filePath("三角网格.ply");
                writeMesh(path, *mesh);
                check(QFileInfo(path).size() > 50000, "mesh file size");
            });
        test("Invalid arguments fail without changing input",
            [&]
            {
                bool rejected = false;
                try
                {
                    execute(Operation::PassThrough, scene, {2, 1, 0, 0});
                }
                catch (...)
                {
                    rejected = true;
                }
                check(rejected, "invalid bounds accepted");
                rejected = false;
                try
                {
                    execute(Operation::Icp, scene, defaults(Operation::Icp));
                }
                catch (...)
                {
                    rejected = true;
                }
                check(rejected, "missing ICP target accepted");
            });
        test("Qt UI: selection, parameter, worker, undo, reset, screenshot",
            [&]
            {
                MainWindow window;
                window.resize(1640, 1000);
                window.show();
                QApplication::processEvents();
                auto list = window.findChild<QListWidget*>("algorithmList");
                auto button = window.findChild<QPushButton*>("runButton");
                auto parameter = window.findChild<QDoubleSpinBox*>("parameter0");
                check(list && button && parameter && list->count() == 15, "UI controls");
                auto originalView = window.findChild<PointCloudView*>("originalView");
                auto resultView = window.findChild<PointCloudView*>("resultView");
                check(originalView && resultView, "point cloud views missing");
                const double initialYaw = originalView->camera().yaw;
                QMouseEvent press(QEvent::MouseButtonPress,
                    QPointF(100., 100.),
                    QPointF(100., 100.),
                    Qt::LeftButton,
                    Qt::LeftButton,
                    Qt::NoModifier);
                QApplication::sendEvent(originalView, &press);
                QMouseEvent move(QEvent::MouseMove,
                    QPointF(140., 120.),
                    QPointF(140., 120.),
                    Qt::NoButton,
                    Qt::LeftButton,
                    Qt::NoModifier);
                QApplication::sendEvent(originalView, &move);
                check(originalView->camera().yaw != initialYaw &&
                        originalView->camera().yaw == resultView->camera().yaw &&
                        originalView->camera().pitch == resultView->camera().pitch,
                    "dragging did not synchronize both cameras");
                QMouseEvent release(QEvent::MouseButtonRelease,
                    QPointF(140., 120.),
                    QPointF(140., 120.),
                    Qt::LeftButton,
                    Qt::NoButton,
                    Qt::NoModifier);
                QApplication::sendEvent(originalView, &release);
                auto initial = window.currentResult().cloud->size();
                parameter->setValue(.08);
                button->click();
                waitForWindow(window);
                check(window.currentResult().cloud->size() < initial,
                    "run button did not process input");
                window.undo();
                check(window.currentResult().cloud->size() == initial, "undo");
                // 真实选择算法并点击按钮，验证分割 → 聚类的连续处理路径。
                window.selectOperation(Operation::Plane);
                button->click();
                waitForWindow(window);
                window.selectOperation(Operation::Clusters);
                button->click();
                waitForWindow(window);
                check(
                    window.currentResult().colors.size() == window.currentResult().cloud->size() &&
                        !window.currentResult().colors.empty(),
                    "UI pipeline");
                QApplication::processEvents();
                QString screenshot = QFileInfo(reportPath).absolutePath() + "/PCLDemo-preview.png";
                QDir().mkpath(QFileInfo(screenshot).absolutePath());
                check(window.grab().save(screenshot), "UI screenshot save failed");
                window.reset();
                check(window.currentResult().cloud->size() == initial, "reset");
                // 故意将全部点滤掉；撤销必须能恢复到非空输入。
                window.selectOperation(Operation::PassThrough);
                window.findChild<QDoubleSpinBox*>("parameter0")->setValue(20);
                window.findChild<QDoubleSpinBox*>("parameter1")->setValue(21);
                button->click();
                waitForWindow(window);
                check(window.currentResult().cloud->empty(), "empty filtering result");
                window.undo();
                check(window.currentResult().cloud->size() == initial, "undo empty result");
                // 无目标 ICP 的失败必须保留当前数据，并在日志中可见。
                window.selectOperation(Operation::Icp);
                button->click();
                waitForWindow(window);
                check(window.currentResult().cloud->size() == initial, "failed task changed data");
                check(window.findChild<QPlainTextEdit*>("logView")->toPlainText().contains("失败"),
                    "failure was not logged");
                window.close();
                QApplication::processEvents();
            });
        runCvSelfTests(test, reportPath);

        report << QString("\n%1 passed, %2 failed").arg(passes).arg(failures);
        QDir().mkpath(QFileInfo(reportPath).absolutePath());
        QSaveFile file(reportPath);
        if (!file.open(QIODevice::WriteOnly))
        {
            return 2;
        }
        auto bytes = report.join('\n').toUtf8();
        if (file.write(bytes) != bytes.size() || !file.commit())
        {
            return 2;
        }
        return failures == 0 ? 0 : 1;
    }
} // namespace demo
