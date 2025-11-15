// main.cpp

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QVBoxLayout>
#include <QSerialPort>
#include <QThread>
#include <QtMath>

// ?? ??? ???
struct LidarPoint {
    float angle;    // degrees
    float distance; // mm
};

// ????? ??? pointReady ???? ??? ??
class LidarWorker : public QObject {
    Q_OBJECT
public:
    explicit LidarWorker(const QString &portName, QObject *parent = nullptr)
      : QObject(parent), serial(this)
    {
        serial.setPortName(portName);
        serial.setBaudRate(QSerialPort::Baud460800);
        serial.setDataBits(QSerialPort::Data8);
        serial.setParity(QSerialPort::NoParity);
        serial.setStopBits(QSerialPort::OneStop);
        serial.setFlowControl(QSerialPort::NoFlowControl);
    }

public slots:
    void startScanning() {
        if (!serial.open(QIODevice::ReadWrite))
            return;
        // SCAN ?? ??
        QByteArray cmd;
        cmd.append(char(0xA5));
        cmd.append(char(0x20));
        serial.write(cmd);
        parseLoop();
        serial.close();
        emit finished();
    }

signals:
    void pointReady(const LidarPoint &pt);
    void finished();

private:
    QSerialPort serial;

    void parseLoop() {
        // Measurement packet: 5???
        while (serial.waitForReadyRead(100)) {
            QByteArray chunk = serial.read(5);
            if (chunk.size() < 5) continue;
            quint8 b0 = quint8(chunk[0]);
            // ?? ??? ??
            if ((b0 & 0x01) != 0x01) continue;

            // angle ??
            quint16 a = (quint8(chunk[2]) | (quint8(chunk[3]) << 8)) >> 1;
            float angle = a / 64.0f;
            // distance ??
            quint16 d = (quint8(chunk[4]) | (quint8(chunk[5]) << 8));
            float dist = d / 4.0f;

            emit pointReady({angle, dist});
        }
    }
};

// ?? ??? (?? + ??? ?)
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr)
      : QMainWindow(parent), scene(new QGraphicsScene(this))
    {
        auto *central = new QWidget(this);
        auto *layout = new QVBoxLayout(central);
        auto *btn = new QPushButton("Start Scanning", central);
        auto *view = new QGraphicsView(scene, central);
        layout->addWidget(view);
        layout->addWidget(btn);
        setCentralWidget(central);

        connect(btn, &QPushButton::clicked, this, &MainWindow::onStartButton);
    }

private slots:
    void onStartButton() {
        // ?? ??? ??
        auto *worker = new LidarWorker("/dev/ttyUSB0");
        auto *thread = new QThread(this);
        worker->moveToThread(thread);
        connect(thread, &QThread::started, worker, &LidarWorker::startScanning);
        connect(worker, &LidarWorker::pointReady, this, &MainWindow::onPoint);
        connect(worker, &LidarWorker::finished, thread, &QThread::quit);
        connect(thread, &QThread::finished, worker, &QObject::deleteLater);
        connect(thread, &QThread::finished, thread, &QObject::deleteLater);
        thread->start();
    }

    void onPoint(const LidarPoint &pt) {
        // Polar ? Cartesian
        float rad = qDegreesToRadians(pt.angle);
        float x = pt.distance * qCos(rad);
        float y = pt.distance * qSin(rad);
        // ?? ??? ??
        auto *dot = scene->addEllipse(x-2, y-2, 4, 4);
        dot->setBrush(Qt::blue);
        dots.append(dot);
        if (dots.size() > 2000) {
            scene->removeItem(dots.first());
            delete dots.first();
            dots.pop_front();
        }
    }

private:
    QGraphicsScene *scene;
    QVector<QGraphicsEllipseItem*> dots;
};

#include "main.moc"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow w;
    w.resize(600, 600);
    w.show();
    return app.exec();
}
