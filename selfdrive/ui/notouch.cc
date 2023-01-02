#include <sys/resource.h>

#include <QApplication>
#include <QtWidgets>
#include <QGraphicsScene>

#include "system/hardware/hw.h"
#include "cereal/messaging/messaging.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/notouch.h"
#include "selfdrive/ui/qt/qt_window.h"

const QString VIDEOS_PATH = "../assets/videos/out";
const QString GST_VIDEO_CMD = QString("while true; do gst-launch-1.0 -v filesrc location=\"%1\" ! decodebin ! videorate ! queue2 ! video/x-raw,framerate=20/1 ! queue2 ! videoconvert ! queue2 ! videoflip method=clockwise ! queue2 ! autovideosink; done");


MainWindowNoTouch::MainWindowNoTouch(QWidget *parent) : QWidget(parent) {
  setpriority(PRIO_PROCESS, 0, -5);
  Hardware::set_brightness(80);

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(0);

  QDir videos_path = QDir(VIDEOS_PATH, "*.mp4");
  QString content_name;
  while ((content_name = MultiOptionDialog::getSelection(tr("Select content"), videos_path.entryList(), "", this)).isEmpty()) {
    qDebug() << "No content selected!";
  }

  content_name = VIDEOS_PATH + "/" + content_name;
  qDebug() << "Selected:" << content_name;

  // play video
  QTimer::singleShot(0, [=]() {
    std::system(QString("while true; do %1; done").arg(GST_VIDEO_CMD.arg(content_name)).toStdString().c_str());
  });

  // no outline to prevent the focus rectangle
  setStyleSheet(R"(
    * {
      font-family: Inter;
      outline: none;
    }
  )");
  setAttribute(Qt::WA_NoSystemBackground);
}

int main(int argc, char *argv[]) {
  initApp(argc, argv);
  QApplication a(argc, argv);
  MainWindowNoTouch w;
  setMainWindow(&w);

  return a.exec();
}