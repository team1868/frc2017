#ifndef SRC_AUTO_VISION_ZMQTEST_H_
#define SRC_AUTO_VISION_ZMQTEST_H_

#include <Auto/Vision/zmq.hpp>
#include <string>
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
	zmq::socket_t *socket;
};

#endif /* SRC_AUTO_VISION_ZMQTEST_H_ */
