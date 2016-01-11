#ifndef _TIMER_H_
#define _TIMER_H_


#include <Windows.h>
#include <iostream>
#include <stdio.h>

class Timer
{
private :
	INT64 startF, endF;
	INT64 frequency;

	float executionT;

public :

	void start();
	float finish();

};

#endif
