#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>

class WatchDog
{
public:
    WatchDog(const char *path);
    void feed(void);
private:
    int fd;
    char buf[3] = {'1','\r','\n'};
};


WatchDog::WatchDog(const char *path)
{
    mkfifo(path,0666);
    fd = open(path,O_WRONLY);
    ftruncate(fd,0);
    lseek(fd,0,SEEK_SET);
}

void WatchDog::feed(void){
    write(fd,buf,3);
}

#endif // WATCHDOG_H