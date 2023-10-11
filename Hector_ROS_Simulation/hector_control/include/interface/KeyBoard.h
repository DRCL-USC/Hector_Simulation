#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "CmdPanel.h"
#include "../common/cppTypes.h"

class KeyBoard : public CmdPanel{
public:
    KeyBoard();
    ~KeyBoard();
private:
    static void *runKeyBoard(void *arg);
    void *run(void *arg);
    UserCommand checkCmd();
    void changeValue();

    pthread_t _tid;
    float sensitivityLeft = 0.025;
    float sensitivityRight = 0.025;
    struct termios _oldSettings, _newSettings;
    fd_set set;
    int res;
    char _c;
};

#endif  // KEYBOARD_H