#pragma once
#include "autorace_lane_fusion/lane.hpp"
#include <QImage>
#include <QWidget>
#include <functional>

namespace lane_lite {
// Main-thread-only image view. Coordinates always refer to the full source
// image, regardless of letterboxing, widget size or display scaling.
class RoiView : public QWidget {
public:
  explicit RoiView(QWidget *parent = nullptr);
  void setImage(const QImage &image);
  void setConfig(const Config &config);
  void setBevEditing(bool enabled);
  QRectF imageRect() const;
  QRectF roiRect() const;
  std::function<void(double left, double top, double right, double bottom)>
      onRoiChanged;
  std::function<void(const Config &)> onBevChanged;

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *) override;
  void mouseMoveEvent(QMouseEvent *) override;
  void mouseReleaseEvent(QMouseEvent *) override;
  void leaveEvent(QEvent *) override;

private:
  enum Edge { None = 0, Left = 1, Top = 2, Right = 4, Bottom = 8 };
  int hitTest(const QPointF &point) const;
  void setHandleCursor(int edges);
  QImage image_;
  Config config_;
  int dragged_ = None;
  bool bev_editing_ = false;
};
} // namespace lane_lite
