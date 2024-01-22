#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "TCP.h"
#include "qcustomplot.h"
#include <QDebug>
#include <QElapsedTimer>
#include <QFile>
#include <QList>
#include <QMainWindow>
#include <QObject>
#include <QTextStream>
#include <QThread>
#include <QVector>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private slots:
  void init_window();

  void on_btConnect_clicked();

  void rxMessageThread(const QList<QByteArray> resMess);

  void plotRespond();

  void on_btClearpl_clicked();

  void on_btTrap_clicked();

private:
  Ui::MainWindow *ui;

  QElapsedTimer time;
  QList<double> timeBuff, velVal1Buff, velVal2Buff, velRefBuff, posVal1Buff,
      posVal2Buff, posRefBuff;
  quint64 preTime = 0;
  TCP tcpThread;
  void plotSetting(QCustomPlot *plot, const char *xLabel, const char *yLabel);
};
#endif // MAINWINDOW_H
