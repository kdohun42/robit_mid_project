#include <QApplication>
#include <iostream>

#include "../include/ebimu/main_window.hpp"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  e2box_imu::MainWindow w(argc, argv);
  w.show();

  return a.exec();
}
