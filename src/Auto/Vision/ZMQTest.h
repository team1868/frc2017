#ifndef SRC_AUTO_VISION_ZMQTEST_H_
#define SRC_AUTO_VISION_ZMQTEST_H_

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

class ZMQTest {
public:
	ZMQTest();
	void Update();
	virtual ~ZMQTest();

private:
	zmq::context_t *context;
	zmq::socket_t *subscriber;
};

#endif /* SRC_AUTO_VISION_ZMQTEST_H_ */
