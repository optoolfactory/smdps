#include "selfdrive/ui/qt/maps/map_eta.h"

#include <QDateTime>
#include <QPainter>

#include "selfdrive/ui/ui.h"

const float MANEUVER_TRANSITION_THRESHOLD = 10;

MapETA::MapETA(QWidget *parent) : QWidget(parent) {
  setVisible(false);
  setAttribute(Qt::WA_TranslucentBackground);
  eta_doc.setUndoRedoEnabled(false);
  eta_doc.setDefaultStyleSheet("body {font-family:Inter;font-size:60px;color:white;} b{font-size:70px;font-weight:600}");
}

void MapETA::paintEvent(QPaintEvent *event) {
  if (!eta_doc.isEmpty()) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 150));
    QSizeF txt_size = eta_doc.size();
    p.drawRoundedRect((width() - txt_size.width()) / 2 - UI_BORDER_SIZE, 0, txt_size.width() + UI_BORDER_SIZE * 2, height() + 25, 25, 25);
    p.translate((width() - txt_size.width()) / 2, (height() - txt_size.height()) / 2);
    eta_doc.drawContents(&p);
  }
}

void MapETA::updateETA(float s, float s_typical, float d) {
  // ETA
  auto eta_t = QDateTime::currentDateTime().addSecs(s).time();
  auto eta = format_24h ? std::array{eta_t.toString("HH:mm"), tr("eta")}
                        : std::array{eta_t.toString("h:mm a").split(' ')[0], eta_t.toString("a")};

  // Remaining time
  auto remaining = s < 3600 ? std::array{QString::number(int(s / 60)), tr("min")}
                            : std::array{QString("%1:%2").arg((int)s / 3600).arg(((int)s % 3600) / 60, 2, 10, QLatin1Char('0')), tr("hr")};
  QString color = "#25DA6E";
  if (s / s_typical > 1.5)
    color = "#DA3025";
  else if (s / s_typical > 1.2)
    color = "#DAA725";

  // Distance
  float num = uiState()->scene.is_metric ? (d / 1000.0) : (d * METER_TO_MILE);
  auto distance = std::array{QString::number(num, 'f', num < 100 ? 1 : 0),
                             uiState()->scene.is_metric ? tr("km") : tr("mi")};

  eta_doc.setHtml(QString(R"(<body><b>%1</b>%2 <span style="color:%3"><b>%4</b>%5</span> <b>%6</b>%7</body>)")
                      .arg(eta[0], eta[1], color, remaining[0], remaining[1], distance[0], distance[1]));

  setVisible(d >= MANEUVER_TRANSITION_THRESHOLD);
  update();
}
