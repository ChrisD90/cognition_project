#ifndef cognition_project_QNODE_HPP_
#define cognition_project_QNODE_HPP_

#include <ros/ros.h>
#include <ros/network.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <QThread>
#include <QString>
#include <QStringList>
#include <QStringListModel>
#include <QSettings>
#include <sstream>
#include <string>


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define NO_IP "000.000.000.000"
#define PORT 5050
#define SIZE 14
#define SEC 1000000000
#define HALF_SEC 500000000
#define MILLI_SEC 1000000

struct SensorData {
	uint32_t timestamp;
	float q0, q1, q2, q3, exInt, eyInt, ezInt;
}__attribute__((packed));

namespace cognition_project {

class QNode : public QThread {
    Q_OBJECT

public:
	QNode(int argc, char** argv);
	virtual ~QNode();

	void resetAddresses();
	void readAddresses();
	void writeAddresses();

	void initMessages();
	void initAddresses();

	bool receiveReady();
	bool nodeReady();
	bool socketReady();
	bool socketCreation();
	bool socketOption();
	bool socketBinding();

	void startThread();
	void stopThread();
	void run();

	std::string node_host_ip;
	std::string node_master_uri;

	enum LogLevel {
	         Info,
	         Error,
	         Data
	 };

	void log(const LogLevel &level, const std::string &msg);
	QStringListModel* loggingModel() { return &logging_model; }
	QStringList* getAddressList() { return &addressList; }


Q_SIGNALS:
	void loggingUpdated();

private:

	QStringList addressList;
	QStringListModel logging_model;

	bool stopIt;

	std::string parent[SIZE];
	std::string child[SIZE];
	tf::Vector3 origin[SIZE];
	tf::Quaternion rotation[SIZE];
	tf::StampedTransform tfMsgs[SIZE];

	//to determine the last time a packets was received for a frame
	ros::Time tfLastUpdate[SIZE];

	//time difference between now and the last time a packet was received for a frame in seconds
	int tfAgeInSec[SIZE];

	//time difference between now and the last time a packet was received for a frame in nanoseconds
	int tfAgeInNSec[SIZE];


	int sock; //socket
	int sockbind; //socket binding
	int sockopt; //socket option
	struct sockaddr_in socketAddr; //socket address
	struct sockaddr_in sensorAddr; //sensor address
	socklen_t sensorAddrLength; //sensor address length
	SensorData packetData; //sensor data
	int packetSize; //received bytes for each packet

};

}

#endif
