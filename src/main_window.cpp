#include "../include/cognition_project/main_window.hpp"

namespace cognition_project {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
		QMainWindow(parent), qnode(argc, argv) {
	ui.setupUi(this);

	ReadSettings();

	ui.list_view_receive_info->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
			SLOT(updateLoggingView()));
}

MainWindow::~MainWindow() {
}

void MainWindow::on_button_receive_stop_clicked() {

	qnode.stopThread();
	ui.button_receive_start->setEnabled(true);
	ui.button_receive_stop->setEnabled(false);

	ui.line_edit_master_url->setReadOnly(false);
	ui.line_edit_host_url->setReadOnly(false);
	ui.line_edit_master_url->setEnabled(true);
	ui.line_edit_host_url->setEnabled(true);
}

void MainWindow::on_button_receive_start_clicked() {

	if (qnode.initNode(ui.line_edit_master_url->text().toStdString(),
			ui.line_edit_host_url->text().toStdString())) {

		qnode.startThread();
		ui.button_receive_start->setEnabled(false);
		ui.button_receive_stop->setEnabled(true);

		ui.line_edit_master_url->setReadOnly(true);
		ui.line_edit_host_url->setReadOnly(true);
		ui.line_edit_master_url->setEnabled(false);
		ui.line_edit_host_url->setEnabled(false);
	}
}

void MainWindow::updateLoggingView() {
	ui.list_view_receive_info->scrollToBottom();
}

void MainWindow::ReadSettings() {
	QSettings settings("Qt-Ros Package", "cognition_project");
	restoreGeometry(settings.value("geometry").toByteArray());
	restoreState(settings.value("windowState").toByteArray());
	QString master_url = settings.value("master_url",
			QString("http://192.168.178.44:11311/")).toString();
	QString host_url =
			settings.value("host_url", QString("192.168.178.44")).toString();
	ui.line_edit_master_url->setText(master_url);
	ui.line_edit_host_url->setText(host_url);

	bool remember = settings.value("remember_settings", false).toBool();
	ui.checkbox_remember_settings->setChecked(remember);
}

void MainWindow::WriteSettings() {
	QSettings settings("Qt-Ros Package", "cognition_project");
	settings.setValue("master_url", ui.line_edit_master_url->text());
	settings.setValue("host_url", ui.line_edit_host_url->text());
	settings.setValue("geometry", saveGeometry());
	settings.setValue("windowState", saveState());
	settings.setValue("remember_settings",
			QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event) {
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}

