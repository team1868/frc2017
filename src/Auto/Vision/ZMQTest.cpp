#include <Auto/Vision/ZMQTest.h>

// EXAMPLE from http://zguide.zeromq.org/cpp:hwserver

ZMQTest::ZMQTest() {
	// TODO Auto-generated constructor stub
	//  Prepare our context and socket
	context = new zmq::context_t(1);
	socket = new zmq::socket_t(*context, ZMQ_REP);
	socket->bind ("tcp://*:5555");
}

void ZMQTest::Update() {
	zmq::message_t request;

	//  Wait for next request from client
	socket->recv (&request);
	std::cout << "Received Hello" << std::endl;

	//  Do some 'work'
	sleep(1);

	//  Send reply back to client
	zmq::message_t reply (5);
	memcpy (reply.data (), "World", 5);
	socket->send (reply);
}

ZMQTest::~ZMQTest() {

}

