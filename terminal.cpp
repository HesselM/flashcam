//
//  terminal.cpp
//  
//
//  Created by Hessel van der Molen on 17/05/17.
//
//

#include "terminal.h"

#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>


void terminal_clear( void ) {
    // Disable output Buffering
    setvbuf(stdout, NULL, _IONBF, 0);
    // Setup input properties
    terminal_setEcho(STDIN_FILENO, 0);
    terminal_setBlocking(STDIN_FILENO, 0);
    terminal_enableBuffer(STDIN_FILENO, 0);
    HIDECURS;
}

void terminal_restore( void ) {
    terminal_setEcho(STDIN_FILENO, 1);
    terminal_setBlocking(STDIN_FILENO, 1);
    terminal_enableBuffer(STDIN_FILENO, 1);
    SHOWCURS;
    
}

void terminal_setEcho(int fd, int enable) {
	   //get current settings
    struct termios settings;
    tcgetattr(fd, &settings);
    //enable or disable?
    if (enable)
        // enable buffer
        settings.c_lflag |= ECHO;
    else
        // disable buffer
        settings.c_lflag &= ~ECHO;
    //apply settings
    tcsetattr(fd, TCSANOW, &settings);
}

void terminal_setBlocking(int fd, int block) {
    int opts = fcntl(fd, F_GETFL);
    if (block) 
        //set blocking
        fcntl(fd, F_SETFL, opts & ~O_NONBLOCK);        
    else
        //set non blocking
        fcntl(fd, F_SETFL, opts | O_NONBLOCK);
}

void terminal_enableBuffer(int fd, int enable) {
    //get current settings
    struct termios settings;
    tcgetattr(fd, &settings);
    //enable or disable?
    if (enable)
        // enable buffer
        settings.c_lflag |= ICANON;
    else
        // disable buffer
        settings.c_lflag &= ~ICANON;
    //apply settings
    tcsetattr(fd, TCSANOW, &settings);
}
