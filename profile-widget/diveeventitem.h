// SPDX-License-Identifier: GPL-2.0
#ifndef DIVEEVENTITEM_H
#define DIVEEVENTITEM_H

#include <QCoreApplication> // for Q_DECLARE_TR_FUNCTIONS
#include <QGraphicsPixmapItem>

class DiveCartesianAxis;
class DivePixmaps;
struct event;
struct plot_info;

class DiveEventItem : public QGraphicsPixmapItem {
	Q_DECLARE_TR_FUNCTIONS(DiveEventItem)
public:
	DiveEventItem(const struct dive *d, struct event *ev, struct gasmix lastgasmix,
		      const struct plot_info &pi, DiveCartesianAxis *hAxis, DiveCartesianAxis *vAxis,
		      int speed, const DivePixmaps &pixmaps, QGraphicsItem *parent = nullptr);
	~DiveEventItem();
	const struct event *getEvent() const;
	struct event *getEventMutable();
	void eventVisibilityChanged(const QString &eventName, bool visible);
	void setVerticalAxis(DiveCartesianAxis *axis, int speed);
	void setHorizontalAxis(DiveCartesianAxis *axis);
	bool shouldBeHidden();
	static bool isInteresting(const struct dive *d, const struct divecomputer *dc,
				  const struct event *ev, const struct plot_info &pi,
				  int firstSecond, int lastSecond);
	const QString text;
	const QPixmap pixmap;
private:
	QString setupToolTipString(const struct event *ev, struct gasmix lastgasmix);
	QPixmap setupPixmap(const struct event *ev, struct gasmix lastgasmix, const DivePixmaps &pixmaps) const;
	void recalculatePos();
	DiveCartesianAxis *vAxis;
	DiveCartesianAxis *hAxis;
	struct event *ev;
	const struct dive *dive;
	int depth;
};

#endif // DIVEEVENTITEM_H
