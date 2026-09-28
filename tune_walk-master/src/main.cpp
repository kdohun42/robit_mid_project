#include <QApplication>
#include <iostream>

#include "../include/tune_walk/main_window.hpp"

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	tune_walk::MainWindow w;
	w.show();
	return a.exec();
}
