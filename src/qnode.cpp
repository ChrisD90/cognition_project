#include "../include/cognition_project/qnode.hpp"

namespace cognition_project {

/*
 *
 */
QNode::QNode(int argc, char** argv) :
		init_argc(argc), init_argv(argv) {

	stopIt = false;
	sock = -1;
	sockbind = -1;
	sockopt = -1;
	sensorAddrLength = 0;
	packetSize = 0;

}

/*
 *
 */
QNode::~QNode() {
	if (ros::isStarted()) {
		ros::shutdown();
		ros::waitForShutdown();
	}
	wait();
}

/*
 *
 */
bool QNode::initNode(const std::string &masterURL, const std::string &hostURL) {

	if (ros::isStarted()) {

		log(Info, std::string("Node already running"));
		return true;
	}

	std::map<std::string, std::string> remappings;
	remappings["__master"] = masterURL;
	remappings["__hostname"] = hostURL;

	ros::init(remappings, "qnode");

	if (!ros::master::check()) {

		log(Error, std::string("ROS Master not found"));
		return false;
	}

	if(!initSocket()) {
		return false;
	}

	ros::start();
	ros::NodeHandle nh;

	initMessages();

	log(Info, std::string("Node was started"));
	return true;
}

/*
 *
 */
bool QNode::initSocket() {

	if(sock >= 0 && sockbind >= 0 && sockopt >= 0) {
		return true;
	}

	//create socket
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		log(Error, std::string("Creation of socket has failed"));
		return false;
	} else {
		log(Info, std::string("Creation of socket was successful"));
	}

	//set socket address to 0
	memset((char *) &socketAddr, 0, sizeof(socketAddr));

	//fill socket address
	socketAddr.sin_family = AF_INET;
	socketAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	socketAddr.sin_port = htons(PORT);

	//bind socket to port
	sockbind = bind(sock, (struct sockaddr *) &socketAddr, sizeof(socketAddr));
	if (sockbind < 0) {
		log(Error, std::string("Binding of socket has failed"));
		return false;
	} else {
		log(Info, std::string("Binding of socket was successful"));
	}

	//set socket option (allow other sockets to bind to this port)
	int optval = 1;
	sockopt = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval,
			sizeof optval);
	if (sockopt < 0) {
		log(Error, std::string("Setting option of socket has failed"));
		return false;
	} else {
		log(Info, std::string("Setting option of socket was successful"));
	}

	//sensor address length
	sensorAddrLength = sizeof(sensorAddr);

	//set sensor address to 0
	memset((char *) &sensorAddr, 0, sizeof(sensorAddr));

	return true;
}

/*
 *
 */

void QNode::initMessages() {

	//URDF Joints
	//************************************
	origin[0] = tf::Vector3(0, 0, 0);
	origin[1] = tf::Vector3(0, 0, 1);
	parent[0] = "base_link";
	child[0] = "link_00_b_body";
	parent[1] = "link_00_b_body";
	child[1] = "link_01_b_head";
	//************************************
	origin[2] = tf::Vector3(0, 0.5, 0.95);
	origin[3] = tf::Vector3(0, 0, -0.5);
	origin[4] = tf::Vector3(0, 0, -0.6);
	parent[2] = "link_00_b_body";
	child[2] = "link_02_l_upper_arm";
	parent[3] = "link_02_l_upper_arm";
	child[3] = "link_03_l_lower_arm";
	parent[4] = "link_03_l_lower_arm";
	child[4] = "link_04_l_hand";
	//************************************
	origin[5] = tf::Vector3(0, 0.2, 0);
	origin[6] = tf::Vector3(0, 0, -0.8);
	origin[7] = tf::Vector3(0, 0, -0.725);
	parent[5] = "link_00_b_body";
	child[5] = "link_05_l_upper_leg";
	parent[6] = "link_05_l_upper_leg";
	child[6] = "link_06_l_lower_leg";
	parent[7] = "link_06_l_lower_leg";
	child[7] = "link_07_l_foot";
	//************************************
	origin[8] = tf::Vector3(0, -0.5, 0.95);
	origin[9] = tf::Vector3(0, 0, -0.5);
	origin[10] = tf::Vector3(0, 0, -0.6);
	parent[8] = "link_00_b_body";
	child[8] = "link_08_r_upper_arm";
	parent[9] = "link_08_r_upper_arm";
	child[9] = "link_09_r_lower_arm";
	parent[10] = "link_09_r_lower_arm";
	child[10] = "link_10_r_hand";
	//************************************
	origin[11] = tf::Vector3(0, -0.2, 0);
	origin[12] = tf::Vector3(0, 0, -0.8);
	origin[13] = tf::Vector3(0, 0, -0.725);
	parent[11] = "link_00_b_body";
	child[11] = "link_11_r_upper_leg";
	parent[12] = "link_11_r_upper_leg";
	child[12] = "link_12_r_lower_leg";
	parent[13] = "link_12_r_lower_leg";
	child[13] = "link_13_r_foot";
	//************************************

	for (unsigned int i = 0; i < SIZE; i++) {

		//initial rotation
		rotation[i] = tf::Quaternion(0, 0, 0, sqrt(1.0));
		rotation[i].normalize();

		//initial time
		ros::Time initialTime = ros::Time::now();

		//initialize TF messages
		tfMsgs[i].frame_id_ = parent[i];
		tfMsgs[i].child_frame_id_ = child[i];
		tfMsgs[i].stamp_ = initialTime;
		tfMsgs[i].setOrigin(origin[i]);
		tfMsgs[i].setRotation(rotation[i]);
	}
}

/*
 *
 */

void QNode::startThread() {
	start();
	log(Info, std::string("Thread started"));
	stopIt = false;

}


/*
 *
 */

void QNode::stopThread() {
	stopIt = true;
}


/*
 *
 */

void QNode::run() {

	tf::TransformBroadcaster tfPublisher;

	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(2.0); // Timeout of 2 seconds

	//receive sensor packets while ROS is ok
	while (ros::ok() && !stopIt) {

		//receive bytes to fill struct
		packetSize = recvfrom(sock, (char*) &packetData, sizeof(packetData), 0,
				(struct sockaddr *) &sensorAddr, &sensorAddrLength);

		//if bytes were received
		if (packetSize > 0) {


			tf::Quaternion original(packetData.q1, packetData.q2, packetData.q3,
					packetData.q0);
			original.normalize();

			//tfMsgs[2].setRotation(original);

			std::stringstream ss;

			ss << original.getX() << original.getY();

			if ((ros::Time::now() - start_time) >= timeout) {
				log(Info, std::string(inet_ntoa(sensorAddr.sin_addr)));
				log(Info, ss.str());
				start_time = ros::Time::now();
			}


			for (unsigned int i = 0; i < SIZE; i++) {

				tfMsgs[i].stamp_ = ros::Time::now();
				tfPublisher.sendTransform(tfMsgs[i]);

			}
		}
	}
	log(Info, std::string("Thread stopped"));
}

void QNode::log(const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(), 1);
	std::stringstream logging_model_msg;
	switch (level) {
	case (Info): {
		ROS_INFO_STREAM(msg);
		logging_model_msg << "[INFO]: " << msg;
		break;
	}
	case (Error): {
		ROS_ERROR_STREAM(msg);
		logging_model_msg << "[ERROR]: " << msg;
		break;
	}
	case (Data): {
		ROS_FATAL_STREAM(msg);
		logging_model_msg << "[DATA]: " << msg;
		break;
	}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
			new_row);
	Q_EMIT loggingUpdated();
}

}
