#include <Auto/Commands/PegAlignCommand.h>

using namespace std;

AlignWithPegCommand::AlignWithPegCommand() {
	// TODO Auto-generated constructor stub
	context = new zmq::context_t(1);
	subscriber = new zmq::socket_t(*context, ZMQ_SUB);
	subscriber->connect("tcp://10.18.68.29:5563");
	subscriber->setsockopt( ZMQ_SUBSCRIBE, "B", 1);
}

void AlignWithPegCommand::Update() {
	string address = s_recv (*subscriber);
	//  Read message contents
	string contents = s_recv (*subscriber);

	cout << "[" << address << "] " << contents << endl;
	this_thread::sleep_for(chrono::milliseconds(20));
}

AlignWithPegCommand::~AlignWithPegCommand() {
	// TODO Auto-generated destructor stub
}
