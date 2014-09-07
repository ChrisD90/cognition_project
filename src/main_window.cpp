#include "../include/cognition_project/main_window.hpp"

namespace cognition_project {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
		QMainWindow(parent), qnode(argc, argv) {


	ui.setupUi(this);
	ReadSettings();
	qnode.node_host_ip = ui.line_edit_host_ip->text().toStdString();
	qnode.node_master_uri = ui.line_edit_master_uri->text().toStdString();

	ui.list_view_info->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
			SLOT(updateLoggingView()));
}

MainWindow::~MainWindow() {
}

void MainWindow::on_button_host_info_clicked() {

	QMessageBox msgBox;
	msgBox.setWindowTitle(QString("Host-IP"));
	msgBox.setText(QString("'ifconfig' -> inet address:___.___.___.___"));
	msgBox.exec();
}

void MainWindow::on_button_master_info_clicked() {
	QMessageBox msgBox;
	msgBox.setWindowTitle(QString("Master-URI"));
	msgBox.setText(QString("'roscore' -> ROS_MASTER_URI=http://Host-IP:_____/"));
	msgBox.exec();
}

void MainWindow::on_button_setup_done_clicked() {

	qnode.node_host_ip = ui.line_edit_host_ip->text().toStdString();
	qnode.node_master_uri = ui.line_edit_master_uri->text().toStdString();

	if (qnode.receiveReady()) {
		ui.button_setup_done->setEnabled(false);
		ui.line_edit_host_ip->setEnabled(false);
		ui.line_edit_master_uri->setEnabled(false);
		ui.button_host_info->setEnabled(false);
		ui.button_master_info->setEnabled(false);

		ui.button_setup_edit->setEnabled(true);
		ui.button_receive_start->setEnabled(true);
	}
}

void MainWindow::on_button_setup_edit_clicked() {

	ui.button_setup_edit->setEnabled(false);
	ui.button_receive_start->setEnabled(false);
	ui.button_receive_stop->setEnabled(false);

	ui.button_setup_done->setEnabled(true);
	ui.line_edit_master_uri->setEnabled(true);
	ui.line_edit_host_ip->setEnabled(true);
	ui.button_host_info->setEnabled(true);
	ui.button_master_info->setEnabled(true);
}


void MainWindow::on_button_receive_stop_clicked() {

	qnode.stopThread();
	ui.button_receive_stop->setEnabled(false);
	ui.button_receive_start->setEnabled(true);
}

void MainWindow::on_button_receive_start_clicked() {

	qnode.startThread();
	ui.button_receive_start->setEnabled(false);
	ui.button_receive_stop->setEnabled(true);
}

void MainWindow::updateLoggingView() {
	ui.list_view_info->scrollToBottom();
}

void MainWindow::ReadSettings() {
	QSettings settings(QString("TU Darmstadt"), QString("cognition_project"));
	restoreGeometry(settings.value(QString("geometry")).toByteArray());
	restoreState(settings.value(QString("windowState")).toByteArray());
	QString host_ip = settings.value(QString("host_ip"), QString("192.168.0.100")).toString();
	QString master_uri = settings.value(QString("master_uri"), QString("http://192.168.0.100:11311/")).toString();
	ui.line_edit_host_ip->setText(host_ip);
	ui.line_edit_master_uri->setText(master_uri);
	ui.button_setup_edit->setEnabled(false);
	ui.button_receive_start->setEnabled(false);
	ui.button_receive_stop->setEnabled(false);
}

void MainWindow::WriteSettings() {
	QSettings settings(QString("TU Darmstadt"), QString("cognition_project"));
	settings.setValue(QString("geometry"), saveGeometry());
	settings.setValue(QString("windowState"), saveState());
	settings.setValue(QString("host_ip"), ui.line_edit_host_ip->text());
	settings.setValue(QString("master_uri"), ui.line_edit_master_uri->text());
}

void MainWindow::closeEvent(QCloseEvent *event) {
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}

