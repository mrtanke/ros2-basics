#include <QApplication>
#include <QLabel>
#include <QString>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    QLabel* label = new QLabel();
    QString msg = QString::fromStdString("Hello, Qt with C++17!");
    label->setText(msg);
    label->show();
    app.exec();

    return 0;
}