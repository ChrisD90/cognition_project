#ifndef cognition_project_MAIN_WINDOW_H
#define cognition_project_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include <QtGui>
#include <iostream>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace cognition_project {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings();
	void WriteSettings();
	void closeEvent(QCloseEvent *event);

public Q_SLOTS:
	void on_button_receive_start_clicked();
	void on_button_receive_stop_clicked();
    void updateLoggingView();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}

#endif
