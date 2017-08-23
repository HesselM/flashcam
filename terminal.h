//
//  terminal.hpp
//  
//
//  Created by Hessel van der Molen on 17/05/17.
//
//

#ifndef terminal_h
#define terminal_h

#include <stdio.h>

//https://en.wikipedia.org/wiki/ANSI_escape_code
#define GOTO(a,b) printf("\033[%d;%dH",a,b)
#define NL        printf("\033[K\r\n")
#define CLEAR     printf("\033[2J")
#define CLEARLINE printf("\033[2K")
#define HIDECURS  printf("\033[?25l")
#define SHOWCURS  printf("\033[?25h")

void terminal_clear( void );
void terminal_restore( void );

void terminal_setEcho( int fd, int enable );
void terminal_setBlocking( int fd, int block );
void terminal_enableBuffer( int fd, int enable );

#define TPOS(a,b) "\033[" #a ";" #b "H"
#define TCNL      "\033[K\n"
#define TCL       "\033[K"


/*
#define TPOS(a,b) ""
#define TCNL      "\n"
#define TCL       "\n"
*/

#endif /* terminal_h */
