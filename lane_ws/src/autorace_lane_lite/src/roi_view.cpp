#include "autorace_lane_lite/roi_view.hpp"
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

namespace lane_lite {
RoiView::RoiView(QWidget *parent) : QWidget(parent) {
  setMouseTracking(true);
  setMinimumSize(240, 180);
  setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  setToolTip("Select ROI or BEV editing above the image. ROI masking never "
             "changes the BEV calibration. BEV top/bottom corners share y.");
}
void RoiView::setImage(const QImage &image) {
  image_ = image;
  update();
}
void RoiView::setBevEditing(bool enabled) {
  bev_editing_ = enabled;
  dragged_ = None;
  unsetCursor();
  update();
}
void RoiView::setConfig(const Config &config) {
  if (config.values != config_.values) {
    config_ = config;
    update();
  }
}
QRectF RoiView::imageRect() const {
  if (image_.isNull())
    return {};
  QSizeF size = image_.size();
  size.scale(contentsRect().size(), Qt::KeepAspectRatio);
  const auto area = QRectF(contentsRect());
  return {area.x() + (area.width() - size.width()) / 2,
          area.y() + (area.height() - size.height()) / 2, size.width(),
          size.height()};
}
QRectF RoiView::roiRect() const {
  auto r = imageRect();
  const double w = r.width() - 1, h = r.height() - 1;
  return {r.x() + config_["roi_left"] * w, r.y() + config_["roi_top"] * h,
          (config_["roi_right"] - config_["roi_left"]) * w,
          (config_["roi_bottom"] - config_["roi_top"]) * h};
}
void RoiView::paintEvent(QPaintEvent *) {
  QPainter painter(this);
  painter.fillRect(rect(), QColor(30, 32, 35));
  if (image_.isNull()) {
    painter.setPen(Qt::white);
    painter.drawText(rect(), Qt::AlignCenter, "Waiting for image");
    return;
  }
  const auto image = imageRect(), roi = roiRect();
  painter.drawImage(image, image_);
  painter.save();
  painter.setClipRect(image);
  QPainterPath shade;
  shade.addRect(image);
  shade.addRect(roi);
  painter.fillPath(shade, QColor(0, 0, 0, 90));
  auto point = [&](double x, double y) {
    return QPointF(image.x() + x * (image.width() - 1),
                   image.y() + y * (image.height() - 1));
  };
  const QPolygonF bev(
      {point(config_["bev_top_left"], config_["bev_top_y"]),
       point(config_["bev_top_right"], config_["bev_top_y"]),
       point(config_["bev_bottom_right"], config_["bev_bottom_y"]),
       point(config_["bev_bottom_left"], config_["bev_bottom_y"])});
  painter.setPen(QPen(QColor(70, 180, 255), 2, Qt::DashLine));
  painter.drawPolygon(bev);
  if (bev_editing_) {
    painter.setBrush(QColor(70, 180, 255));
    for (auto p : bev)
      painter.drawEllipse(p, 6, 6);
  }
  painter.restore();
  painter.setPen(QPen(QColor(255, 210, 0), 2));
  painter.setBrush(Qt::NoBrush);
  painter.drawRect(roi);
  painter.setBrush(QColor(255, 210, 0));
  if (!bev_editing_)
    for (auto p : {roi.topLeft(), QPointF(roi.center().x(), roi.top()),
                   roi.topRight(), QPointF(roi.right(), roi.center().y()),
                   roi.bottomRight(), QPointF(roi.center().x(), roi.bottom()),
                   roi.bottomLeft(), QPointF(roi.left(), roi.center().y())})
      painter.drawRect(QRectF(p.x() - 4, p.y() - 4, 8, 8));
  painter.drawText(QPointF(image.x() + 8, image.y() + 18),
                   bev_editing_ ? "BEV: drag blue corners (shared row y)"
                                : "ROI: drag yellow edges/corners");
}
int RoiView::hitTest(const QPointF &p) const {
  if (image_.isNull() || !imageRect().contains(p))
    return None;
  if (bev_editing_) {
    auto image = imageRect();
    for (int corner :
         {Left | Top, Right | Top, Left | Bottom, Right | Bottom}) {
      const bool top = corner & Top, left = corner & Left;
      double x = config_[top ? (left ? "bev_top_left" : "bev_top_right")
                             : (left ? "bev_bottom_left" : "bev_bottom_right")];
      double y = config_[top ? "bev_top_y" : "bev_bottom_y"];
      QPointF handle(image.x() + x * (image.width() - 1),
                     image.y() + y * (image.height() - 1));
      if (QLineF(handle, p).length() <= 12)
        return corner;
    }
    return None;
  }
  const auto r = roiRect();
  const double tolerance = 10;
  if (!r.adjusted(-tolerance, -tolerance, tolerance, tolerance).contains(p))
    return None;
  int edges = None;
  if (std::abs(p.x() - r.left()) <= tolerance)
    edges |= Left;
  else if (std::abs(p.x() - r.right()) <= tolerance)
    edges |= Right;
  if (std::abs(p.y() - r.top()) <= tolerance)
    edges |= Top;
  else if (std::abs(p.y() - r.bottom()) <= tolerance)
    edges |= Bottom;
  return edges;
}
void RoiView::setHandleCursor(int edges) {
  if (edges == (Left | Top) || edges == (Right | Bottom))
    setCursor(Qt::SizeFDiagCursor);
  else if (edges == (Right | Top) || edges == (Left | Bottom))
    setCursor(Qt::SizeBDiagCursor);
  else if (edges & (Left | Right))
    setCursor(Qt::SizeHorCursor);
  else if (edges & (Top | Bottom))
    setCursor(Qt::SizeVerCursor);
  else
    unsetCursor();
}
void RoiView::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    dragged_ = hitTest(event->localPos());
    if (dragged_) {
      event->accept();
      return;
    }
  }
  QWidget::mousePressEvent(event);
}
void RoiView::mouseMoveEvent(QMouseEvent *event) {
  if (!dragged_) {
    setHandleCursor(hitTest(event->localPos()));
    return;
  }
  const auto image = imageRect();
  if (image.width() < 2 || image.height() < 2)
    return;
  const auto pos = event->localPos();
  double x = std::clamp((pos.x() - image.x()) / (image.width() - 1), 0., 1.);
  double y = std::clamp((pos.y() - image.y()) / (image.height() - 1), 0., 1.);
  // Match three-decimal spin boxes and leave a strict > 0.05 valid span.
  x = std::round(x * 1000) / 1000;
  y = std::round(y * 1000) / 1000;
  if (bev_editing_) {
    const bool top = dragged_ & Top, left = dragged_ & Left;
    const std::string prefix = top ? "bev_top_" : "bev_bottom_";
    Config candidate = config_;
    candidate.values[prefix + (left ? "left" : "right")] =
        left ? std::min(x, config_[prefix + "right"] - .051)
             : std::max(x, config_[prefix + "left"] + .051);
    candidate.values[prefix + "y"] =
        top ? std::min(y, config_["bev_bottom_y"] - .051)
            : std::max(y, config_["bev_top_y"] + .051);
    try {
      candidate.validate();
      config_ = candidate;
      if (onBevChanged)
        onBevChanged(config_);
      update();
    } catch (const std::invalid_argument &) {
      // A drag cannot invert the calibrated polygon.
    }
    event->accept();
    return;
  }
  double l = config_["roi_left"], r = config_["roi_right"],
         t = config_["roi_top"], b = config_["roi_bottom"];
  constexpr double min_span = .051;
  if (dragged_ & Left)
    l = std::min(x, r - min_span);
  if (dragged_ & Right)
    r = std::max(x, l + min_span);
  if (dragged_ & Top)
    t = std::min(y, b - min_span);
  if (dragged_ & Bottom)
    b = std::max(y, t + min_span);
  config_.values["roi_left"] = l;
  config_.values["roi_right"] = r;
  config_.values["roi_top"] = t;
  config_.values["roi_bottom"] = b;
  if (onRoiChanged)
    onRoiChanged(l, t, r, b);
  update();
  event->accept();
}
void RoiView::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton && dragged_) {
    mouseMoveEvent(event);
    dragged_ = None;
    setHandleCursor(hitTest(event->localPos()));
    event->accept();
    return;
  }
  QWidget::mouseReleaseEvent(event);
}
void RoiView::leaveEvent(QEvent *) {
  if (!dragged_)
    unsetCursor();
}
} // namespace lane_lite
