#ifndef SRC_AUTO_COMMANDS_AUTOCOMMAND_H_
#define SRC_AUTO_COMMANDS_AUTOCOMMAND_H_

class AutoCommand {
public:
	AutoCommand() {}
	virtual ~AutoCommand() {}
	virtual void Init() = 0;
	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;
	virtual bool IsDone() = 0;
};

#endif /* SRC_AUTO_COMMANDS_AUTOCOMMAND_H_ */
