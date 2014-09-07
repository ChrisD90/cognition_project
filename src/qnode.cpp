#include "../include/cognition_project/qnode.hpp"

namespace cognition_project {



/*
 *
 */
QNode::QNode(int argc, char** argv) {

	initAddresses();
	initMessages();
	stopIt = false;



	FrameAddress a;
	FrameAddress b;
	FrameAddress c;
	FrameAddress d;

	a.frame = 4;
	a.address = QString("A");
	b.frame = 2;
	b.address = QString("B");
	c.frame = 1;
	c.address = QString("C");
	d.frame = 3;
	d.address = QString("D");

	frameAddressList.append(a);
	frameAddressList.append(b);
	frameAddressList.append(c);
	frameAddressList.append(d);

	QSettings settings(QString("TU Darmstadt"), QString("cognition_project"));

	/*
	settings.beginGroup("frame_address");
		    settings.remove("");
		    settings.endGroup();
		    */


	settings.beginWriteArray("frame_address");

	for (int i = 0; i < frameAddressList.size(); ++i) {
	    settings.setArrayIndex(i);
	    settings.setValue("frame", frameAddressList.at(i).frame);
	    settings.setValue("address", frameAddressList.at(i).address);
	}
	settings.endArray();


	int size = settings.beginReadArray("frame_address");
	for (int i = 0; i < size; ++i) {
	    settings.setArrayIndex(i);
	    FrameAddress fa;
	    fa.frame = settings.value("frame").toInt();
	    fa.address = settings.value("address").toString();
	    frameAddressList.append(fa);
	}
	settings.endArray();

	for (int i = 0; i < frameAddressList.size(); ++i) {
		log(Info, frameAddressList.at(i).address.toStdString());
	}



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
void QNode::initAddresses() {

	sock = -1;
	sockbind = -1;
	sockopt = -1;
	sensorAddrLength = 0;
	packetSize = 0;

	//set socket address to 0
	memset((char *) &socketAddr, 0, sizeof(socketAddr));

	//set sensor address to 0
	memset((char *) &sensorAddr, 0, sizeof(sensorAddr));

	//fill socket address
	socketAddr.sin_family = AF_INET;
	socketAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	socketAddr.sin_port = htons(PORT);

	//sensor address length
	sensorAddrLength = sizeof(sensorAddr);
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

		//initialize TF messages
		tfMsgs[i].frame_id_ = parent[i];
		tfMsgs[i].child_frame_id_ = child[i];
		tfMsgs[i].setOrigin(origin[i]);
		tfMsgs[i].setRotation(rotation[i]);
	}
}

//**************************************************************************************

/*
 *
 */
bool QNode::receiveReady() {

	if(!nodeReady()) {
		return false;
	}
	if (!socketReady()) {
		return false;
	}
	return true;
}

/*
 *
 */
bool QNode::nodeReady() {

	if(ros::isStarted()) {
		log(Info, std::string("Node is ready"));
		return true;
	}

	std::map<std::string, std::string> remappings;
	remappings["__hostname"] = node_host_ip;
	remappings["__master"] = node_master_uri;

	ros::init(remappings, "qnode");

	if (!ros::master::check()) {
		log(Error, std::string("Master communication failed!"));
		log(Info, std::string("Restart with correct setup."));
		return false;
	}
	ros::start();
	ros::NodeHandle nh;

	log(Info, std::string("Node is ready"));
	return true;
}

/*
 *
 */
bool QNode::socketReady() {

	if(!socketCreation()) {
		log(Error, std::string("Socket creation failed!"));
		return false;
	}
	if(!socketOption()) {
		log(Error, std::string("Socket option setting failed!"));
		return false;
	}
	if(!socketBinding()) {
		log(Error, std::string("Socket binding failed!"));
		return false;
	}
	log(Info, std::string("Socket is ready"));
	return true;
}


/*
 *
 */
bool QNode::socketCreation() {

	if(!(sock < 0)) {
		return true;
	}

	//create socket
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		return false;
	}
	return true;
}

/*
 *
 */
bool QNode::socketOption() {

	if(!(sockopt< 0)) {
		return true;
	}

	//set socket option (allow other sockets to bind to this port)
	int optval = 1;
	sockopt = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval,
			sizeof optval);
	if (sockopt < 0) {
		return false;
	}
	return true;
}

/*
 *
 */
bool QNode::socketBinding() {

	if(!(sockbind < 0)) {
		return true;
	}

	//bind socket to port
	sockbind = bind(sock, (struct sockaddr *) &socketAddr, sizeof(socketAddr));

	if (sockbind < 0) {
		return false;
	}
	return true;
}

//**************************************************************************************

/*
 *
 */
void QNode::startThread() {
	stopIt = false;
	start();
	log(Info, std::string("Started to receive"));
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
	log(Info, std::string("Stopped to receive"));
}

//**************************************************************************************

/*
 *
 */
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
