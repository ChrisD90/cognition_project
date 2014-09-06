#ifndef cognition_project_QNODE_HPP_
#define cognition_project_QNODE_HPP_

#include <ros/ros.h>
#include <ros/network.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <QThread>
#include <QStringListModel>
#include <sstream>
#include <string>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define PORT 5050
#define SIZE 14

struct sensor_data {
	uint32_t timestamp;
	float q0, q1, q2, q3, exInt, eyInt, ezInt;
}__attribute__((packed));


namespace cognition_project {

class QNode : public QThread {
    Q_OBJECT

public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	//bool socketStatus;
	bool initNode(const std::string &master_url, const std::string &host_url);
	bool initSocket();
	void initMessages();
	void startThread();
	void stopThread();
	void run();

	enum LogLevel {
	         Info,
	         Error,
	         Data
	 };

	void log(const LogLevel &level, const std::string &msg);
	QStringListModel* loggingModel() { return &logging_model; }


Q_SIGNALS:
	void loggingUpdated();

private:
	int init_argc;
	char** init_argv;

	 QStringListModel logging_model;


	bool stopIt;

	std::string parent[SIZE];
	std::string child[SIZE];
	tf::Vector3 origin[SIZE];
	tf::Quaternion rotation[SIZE];
	tf::StampedTransform tfMsgs[SIZE];


	int sock; //socket
	int sockbind; //socket binding
	int sockopt; //socket option
	struct sockaddr_in socketAddr; //socket address
	struct sockaddr_in sensorAddr; //sensor address
	socklen_t sensorAddrLength; //sensor address length
	sensor_data packetData; //sensor data
	int packetSize; //received bytes for each packet

};

}

#endif
