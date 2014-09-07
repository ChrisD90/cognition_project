#ifndef cognition_project_MAIN_WINDOW_H
#define cognition_project_MAIN_WINDOW_H

#include <QtGui>
#include <QList>
#include <QtGui/QMainWindow>
#include <QMessageBox>
#include <iostream>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace cognition_project {

struct FrameAddress {
	int frame;
	QString address;
};

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings();
	void WriteSettings();
	void closeEvent(QCloseEvent *event);

public Q_SLOTS:
	void on_button_host_info_clicked();
	void on_button_master_info_clicked();
	void on_button_setup_done_clicked();
	void on_button_setup_edit_clicked();
	void on_button_receive_start_clicked();
	void on_button_receive_stop_clicked();
    void updateLoggingView();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	std::string host_ip;
	std::string master_uri;


};

}

#endif
