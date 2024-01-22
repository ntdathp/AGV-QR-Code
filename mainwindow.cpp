#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  init_window();

  connect(&tcpThread, &TCP::response, this, &MainWindow::rxMessageThread);

  ui->txtIP->setText("192.168.4.1");
  ui->txtPort->setText("8880");
}

void MainWindow::init_window() {
  setFixedSize(width(), height());
  setWindowTitle("Movement Test via TCP");
  ui->btTrap->setDisabled(true);
  ui->btCmd->setDisabled(true);

  ui->btConnect->setText("Connect");
  ui->btConnect->setStyleSheet("QPushButton {color: green;}");

  ui->cbL->setChecked(true);
  ui->cbR->setChecked(true);

  QPen pen1;
  pen1.setStyle(Qt::SolidLine);
  pen1.setWidth(3);
  pen1.setColor("#E2483E");

  QPen pen2;
  pen2.setStyle(Qt::SolidLine);
  pen2.setWidth(3);
  pen2.setColor("#5C97E3");

  QPen pen3;
  pen3.setStyle(Qt::SolidLine);
  pen3.setWidth(3);
  pen3.setColor("#5CE36E");

  // vel plot init
  ui->plVel->addGraph();
  ui->plVel->graph(0)->setLineStyle(QCPGraph::lsLine);
  ui->plVel->graph(0)->setPen(pen1);
  ui->plVel->graph(0)->setName("Reference");

  ui->plVel->addGraph();
  ui->plVel->graph(1)->setLineStyle(QCPGraph::lsLine);
  ui->plVel->graph(1)->setPen(pen2);
  ui->plVel->graph(1)->setName("Velocity 1");

  ui->plVel->addGraph();
  ui->plVel->graph(2)->setLineStyle(QCPGraph::lsLine);
  ui->plVel->graph(2)->setPen(pen3);
  ui->plVel->graph(2)->setName("Velocity 2");

  plotSetting(ui->plVel, "Time", "Velocity");

  // pos plot init
  ui->plPos->addGraph();
  ui->plPos->graph(0)->setLineStyle(QCPGraph::lsLine);
  ui->plPos->graph(0)->setPen(pen1);
  ui->plPos->graph(0)->setName("Reference");

  ui->plPos->addGraph();
  ui->plPos->graph(1)->setLineStyle(QCPGraph::lsLine);
  ui->plPos->graph(1)->setPen(pen2);
  ui->plPos->graph(1)->setName("Position 1");

  ui->plPos->addGraph();
  ui->plPos->graph(2)->setLineStyle(QCPGraph::lsLine);
  ui->plPos->graph(2)->setPen(pen3);
  ui->plPos->graph(2)->setName("Position 2");

  plotSetting(ui->plPos, "Time", "Position");
}

void MainWindow::plotSetting(QCustomPlot *plot, const char *xLabel,
                             const char *yLabel) {
  QFont legendFont = font();
  legendFont.setPointSize(8);
  plot->yAxis->setLabel(yLabel);
  plot->xAxis->setLabel(xLabel);
  plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  plot->legend->setVisible(true);
  plot->legend->setFont(legendFont);
  plot->legend->setSelectedFont(legendFont);
  plot->legend->setSelectableParts(QCPLegend::spItems);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_btConnect_clicked() {
  if (ui->btConnect->text() == "Connect") {
    if (!tcpThread.connect(ui->txtIP->text(), ui->txtPort->text().toInt())) {
      ui->btConnect->setText("Disconnect");
      ui->btConnect->setStyleSheet("QPushButton {color: red;}");
      ui->btTrap->setDisabled(false);
      ui->btCmd->setDisabled(false);
    }
  } else {
    tcpThread.disconnect();
    ui->btConnect->setText("Connect");
    ui->btConnect->setStyleSheet("QPushButton {color: green;}");
    ui->btTrap->setDisabled(true);
    ui->btCmd->setDisabled(true);
  }
}

void MainWindow::rxMessageThread(const QList<QByteArray> resMess) {
  ui->txtTime->append("Received data at time: " +
                      QTime::currentTime().toString());

  quint64 currentTime = time.elapsed() + preTime;
  timeBuff.append(currentTime);
  if (timeBuff.size() >= 8000)
    timeBuff.removeFirst();

  preTime = currentTime;
  time.start();

  velVal1Buff.append(resMess[0].toFloat());
  if (velVal1Buff.size() >= 800)
    velVal1Buff.removeFirst();

  velVal2Buff.append(resMess[1].toFloat());
  if (velVal2Buff.size() >= 800)
    velVal2Buff.removeFirst();

  velRefBuff.append(resMess[2].toFloat());
  if (velRefBuff.size() >= 800)
    velRefBuff.removeFirst();

  posVal1Buff.append(resMess[3].toFloat());
  if (posVal1Buff.size() >= 800)
    posVal1Buff.removeFirst();

  posVal2Buff.append(resMess[4].toFloat());
  if (posVal2Buff.size() >= 800)
    posVal2Buff.removeFirst();

  posRefBuff.append(resMess[5].toFloat());
  if (posRefBuff.size() >= 800)
    posRefBuff.removeFirst();
  plotRespond();
}

void MainWindow::plotRespond() {
  ui->plVel->graph(0)->setData(timeBuff, velRefBuff);
  ui->plVel->graph(1)->setData(timeBuff, velVal1Buff);
  ui->plVel->graph(2)->setData(timeBuff, velVal2Buff);
  ui->plVel->rescaleAxes();
  ui->plVel->replot();
  ui->plVel->update();

  ui->plPos->graph(0)->setData(timeBuff, posRefBuff);
  ui->plPos->graph(1)->setData(timeBuff, posVal1Buff);
  ui->plPos->graph(2)->setData(timeBuff, posVal2Buff);
  ui->plPos->rescaleAxes();
  ui->plPos->replot();
  ui->plPos->update();
}

void MainWindow::on_btClearpl_clicked() {
  time.elapsed();
  timeBuff.clear();
  velVal1Buff.clear();
  velVal2Buff.clear();
  velRefBuff.clear();
  posVal1Buff.clear();
  posVal2Buff.clear();
  posRefBuff.clear();
}

void MainWindow::on_btTrap_clicked() {
  QString TX_Data;
  char cR = '0', cL = '0';
  if (ui->cbR->isChecked()) {
    cR = '1';
  }

  if (ui->cbL->isChecked()) {
    cL = '1';
  }
  QString pos = ui->txtPos->text();
  QString vel = ui->txtVel->text();
  QString acc = ui->txtAcc->text();

  while (pos.length() < 4)
    pos = "0" + pos;
  while (vel.length() < 4)
    vel = "0" + vel;
  while (acc.length() < 4)
    acc = "0" + acc;

  TX_Data.append(pos + "," + vel + "," + acc + "," + cR + "," + cL);
  tcpThread.transmit(TX_Data);
  time.elapsed();
  time.start();
}
