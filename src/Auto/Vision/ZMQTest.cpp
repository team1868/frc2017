#include <Auto/Vision/ZMQTest.h>

ZMQTest::ZMQTest() {
	context = new zmq::context_t(1);
	subscriber = new zmq::socket_t(*context, ZMQ_SUB);
	subscriber->connect("tcp://10.18.68.29:5563");		// IP address of Jetson
	subscriber->setsockopt( ZMQ_SUBSCRIBE, "B", 1);
}

void ZMQTest::Update() {
	std::string address = s_recv (*subscriber);
	std::string contents = s_recv (*subscriber);

	std::cout << "[" << address << "] " << contents << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

ZMQTest::~ZMQTest() {

}
