#include "pointcloudpreviewwidget.h"

#include <QFile>
#include <QMouseEvent>
#include <QPainter>
#include <QRegularExpression>
#include <QStringList>
#include <QTextStream>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>

PointCloudPreviewWidget::PointCloudPreviewWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(760, 560);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}

bool PointCloudPreviewWidget::setCloud(const cv::Mat &points3d, const cv::Mat &colors, QString *errorMessage)
{
    points.clear();

    if (points3d.empty() || colors.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("point cloud or color image is empty");
        }
        update();
        return false;
    }

    if (points3d.type() != CV_32FC3 || colors.type() != CV_8UC3 || points3d.size() != colors.size()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("point cloud matrix and color image do not match");
        }
        update();
        return false;
    }

    const int maxPreviewPoints = 70000;
    const int stride = std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(points3d.total()) / maxPreviewPoints))));

    for (int row = 0; row < points3d.rows; row += stride) {
        const cv::Vec3f *pointRow = points3d.ptr<cv::Vec3f>(row);
        const cv::Vec3b *colorRow = colors.ptr<cv::Vec3b>(row);

        for (int col = 0; col < points3d.cols; col += stride) {
            const cv::Vec3f &rawPoint = pointRow[col];
            const float x = rawPoint[0];
            const float y = -rawPoint[1];
            const float z = rawPoint[2];

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            if (z <= 0.0f || z > 15000.0f || x < -10000.0f || x > 10000.0f || y < -10000.0f || y > 10000.0f) {
                continue;
            }

            const cv::Vec3b &bgr = colorRow[col];
            const QVector3D position(x, y, z);
            points.push_back({position, QColor(bgr[2], bgr[1], bgr[0])});
        }
    }

    return updateBoundsAndView(errorMessage);
}

bool PointCloudPreviewWidget::setCloudFromPly(const QString &fileName, QString *errorMessage)
{
    points.clear();

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("cannot open PLY file");
        }
        update();
        return false;
    }

    QTextStream stream(&file);
    QString line = stream.readLine().trimmed();
    if (line != QStringLiteral("ply")) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("not a PLY file");
        }
        update();
        return false;
    }

    int vertexCount = -1;
    bool asciiFormat = false;
    while (!stream.atEnd()) {
        line = stream.readLine().trimmed();
        if (line == QStringLiteral("format ascii 1.0")) {
            asciiFormat = true;
        } else if (line.startsWith(QStringLiteral("element vertex "))) {
            vertexCount = line.mid(QStringLiteral("element vertex ").length()).toInt();
        } else if (line == QStringLiteral("end_header")) {
            break;
        }
    }

    if (!asciiFormat || vertexCount <= 0) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("only ASCII PLY with vertices is supported");
        }
        update();
        return false;
    }

    const int maxPreviewPoints = 100000;
    const int stride = std::max(1, static_cast<int>(std::ceil(static_cast<double>(vertexCount) / maxPreviewPoints)));

    for (int index = 0; index < vertexCount && !stream.atEnd(); ++index) {
        line = stream.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }

        if (index % stride != 0) {
            continue;
        }

        const QStringList parts = line.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
        if (parts.size() < 6) {
            continue;
        }

        bool okX = false;
        bool okY = false;
        bool okZ = false;
        const float x = parts[0].toFloat(&okX);
        const float y = parts[1].toFloat(&okY);
        const float z = parts[2].toFloat(&okZ);
        if (!okX || !okY || !okZ || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        const int r = std::clamp(parts[3].toInt(), 0, 255);
        const int g = std::clamp(parts[4].toInt(), 0, 255);
        const int b = std::clamp(parts[5].toInt(), 0, 255);
        points.push_back({QVector3D(x, y, z), QColor(r, g, b)});
    }

    return updateBoundsAndView(errorMessage);
}

bool PointCloudPreviewWidget::updateBoundsAndView(QString *errorMessage)
{
    if (points.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("no valid point can be previewed");
        }
        update();
        return false;
    }

    QVector3D minBound(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    QVector3D maxBound(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());

    for (const CloudPoint &point : points) {
        minBound.setX(std::min(minBound.x(), point.position.x()));
        minBound.setY(std::min(minBound.y(), point.position.y()));
        minBound.setZ(std::min(minBound.z(), point.position.z()));
        maxBound.setX(std::max(maxBound.x(), point.position.x()));
        maxBound.setY(std::max(maxBound.y(), point.position.y()));
        maxBound.setZ(std::max(maxBound.z(), point.position.z()));
    }

    center = (minBound + maxBound) * 0.5f;
    radius = std::max(1.0f, (maxBound - minBound).length() * 0.5f);
    zoom = 1.0f;

    update();
    return true;
}

void PointCloudPreviewWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.fillRect(rect(), QColor(18, 21, 25));

    painter.setPen(QColor(218, 224, 232));
    painter.drawText(QRect(12, 10, width() - 24, 24), Qt::AlignLeft | Qt::AlignVCenter,
                     QStringLiteral("Point cloud preview: drag to rotate, wheel to zoom"));

    if (points.isEmpty()) {
        painter.setPen(QColor(170, 180, 190));
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("No point cloud data"));
        return;
    }

    const float yawRad = qDegreesToRadians(yaw);
    const float pitchRad = qDegreesToRadians(pitch);
    const float cy = std::cos(yawRad);
    const float sy = std::sin(yawRad);
    const float cp = std::cos(pitchRad);
    const float sp = std::sin(pitchRad);
    const float scale = std::min(width(), height()) * 0.42f * zoom / radius;
    const QPointF origin(width() * 0.5, height() * 0.55);

    struct PaintPoint {
        float depth;
        QPointF screen;
        QColor color;
    };

    QVector<PaintPoint> paintPoints;
    paintPoints.reserve(points.size());

    for (const CloudPoint &point : points) {
        QVector3D p = point.position - center;

        const float x1 = p.x() * cy + p.z() * sy;
        const float z1 = -p.x() * sy + p.z() * cy;
        const float y1 = p.y() * cp - z1 * sp;
        const float z2 = p.y() * sp + z1 * cp;

        paintPoints.push_back({z2, QPointF(origin.x() + x1 * scale, origin.y() - y1 * scale), point.color});
    }

    std::sort(paintPoints.begin(), paintPoints.end(), [](const PaintPoint &left, const PaintPoint &right) {
        return left.depth > right.depth;
    });

    painter.setPen(Qt::NoPen);
    const float pointSize = zoom > 1.4f ? 2.2f : 1.4f;
    const QRect visible = rect().adjusted(-4, -4, 4, 4);
    for (const PaintPoint &point : paintPoints) {
        if (!visible.contains(point.screen.toPoint())) {
            continue;
        }
        painter.setBrush(point.color);
        painter.drawRect(QRectF(point.screen.x(), point.screen.y(), pointSize, pointSize));
    }
}

void PointCloudPreviewWidget::mousePressEvent(QMouseEvent *event)
{
    lastMousePosition = event->pos();
}

void PointCloudPreviewWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        const QPoint delta = event->pos() - lastMousePosition;
        yaw += delta.x() * 0.45f;
        pitch = std::clamp(pitch + delta.y() * 0.45f, -89.0f, 89.0f);
        lastMousePosition = event->pos();
        update();
    }
}

void PointCloudPreviewWidget::wheelEvent(QWheelEvent *event)
{
    const float steps = event->angleDelta().y() / 120.0f;
    zoom = std::clamp(zoom * std::pow(1.12f, steps), 0.2f, 8.0f);
    update();
}
