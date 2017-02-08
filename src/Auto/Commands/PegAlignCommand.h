#ifndef SRC_AUTO_COMMANDS_PEGALIGNCOMMAND_H_
#define SRC_AUTO_COMMANDS_PEGALIGNCOMMAND_H_

#include <zmq.hpp>
#include <zhelpers.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>

#define sleep(n)    Sleep(n)
#endif

class AlignWithPegCommand {
public:
	AlignWithPegCommand();
	void Init();
	void Update();
	virtual ~AlignWithPegCommand();

private:
	zmq::context_t *context; //(1);
	zmq::socket_t *subscriber; //(context, ZMQ_REP);
};

#endif /* SRC_AUTO_COMMANDS_PEGALIGNCOMMAND_H_ */
