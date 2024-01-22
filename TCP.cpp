#include "TCP.h"

TCP::TCP(QObject *parent) : QThread{parent} {}

bool TCP::connect(QString ip, quint16 port) {
  QObject::connect(&socket, SIGNAL(readyRead()), this, SLOT(receive()));
  socket.connectToHost(QHostAddress(ip), port);

  if (socket.waitForConnected()) {
    if (!isRunning()) {
      start();
    }
    TCPConnect = true;
    qDebug() << "bugg";

    return 0;
  } else {
    emit errorConnect(tr("Can't connect to %1:%2, error code %3")
                          .arg(ip)
                          .arg(port)
                          .arg(socket.error()),
                      Qt::red);
    return 1;
  }
}

void TCP::disconnect() {
  terminate();
  socket.close();
}

void TCP::transmit(QString transData) { socket.write(transData.toUtf8()); }

bool TCP::stateConnect() { return TCPConnect; }

void TCP::receive() {
  QByteArray dataRX_ba = socket.readAll();
  dataRX.append(dataRX_ba);
  int indexBegin = dataRX.indexOf(QString("=").toUtf8());
  int indexEnd = dataRX.indexOf(QString("!").toUtf8());
  qDebug() << dataRX;
  if (indexBegin != -1 && indexEnd != -1) {
    // Both "=" and "!" characters were found
    dataRX = dataRX.mid(indexBegin, indexEnd - indexBegin + 1);
    qDebug() << dataRX;
  }

  QList<QByteArray> buff;

  if (dataRX.startsWith("=") && dataRX.endsWith("!")) {
    dataRX = dataRX.mid(1, dataRX.length() - 2);
    // Tách giá trị A1 đến A6 bằng dấu phẩy và lưu vào danh sách buff
    buff = dataRX.split(',');
    qDebug() << buff;
    // Kiểm tra xem có đúng 6 giá trị trong danh sách
    if (buff.size() == 6) {
      emit response(buff);
      dataRX.clear();
    } else {
      qDebug() << "Not have 6 value";
      qDebug() << dataRX;
    }
  } else {
    qDebug() << "Error on begin or end";
    qDebug() << dataRX;
  }
}

void TCP::run() {}
