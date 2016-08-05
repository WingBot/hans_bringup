/*
 * GeneralFunction.h
 *
 *  Created on: Jul 20, 2016
 *      Author: agv
 */

#ifndef INCLUDE_HANS_BRINGUP_GENERALFUNCTION_H_
#define INCLUDE_HANS_BRINGUP_GENERALFUNCTION_H_


#include <sys/time.h>
#include <sys/select.h>
#include <time.h>
#include <stdio.h>


	/*seconds: the seconds; mseconds: the micro seconds*/
	int Timer(int seconds, int mseconds)
	{
	        struct timeval temp;

	        temp.tv_sec = seconds;
	        temp.tv_usec = mseconds;

	        select(0, NULL, NULL, NULL, &temp);
	        printf("timer\n");

	        return 0;
	}



#endif /* INCLUDE_HANS_BRINGUP_GENERALFUNCTION_H_ */
