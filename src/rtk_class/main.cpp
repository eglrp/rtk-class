#include <cstdio>
#include <iostream>
#include <string>
#include <signal.h>
#include <stdio.h>
#include <conio.h>
#include "gnss.h"
#include "rtklib.h"

using namespace std;


int main(int argc, char **argv)
{

	GNSS gnss_test("../../data/config/rtk.conf");
	gnss_test.initgnss();
	gnss_test.startsvr();
	char control_char;
	while (1) {
		if (kbhit())
		{
			control_char = getch();
			if (control_char == 27)
			{
				break;
			}
		}
		rtklib::obs_t obs;
		rtklib::nav_t nav;
		int obsn;
		gnss_test.getdata(obs,obsn,nav);
		gnss_test.getstatus();
		double pos[6];
		gnss_test.getpos(pos);
		//printf("%f %f %f\n", pos[0], pos[1], pos[2]);
		rtklib::sol_t sol_ = gnss_test.getsol();
		rtklib::sleepms(1000);
	}
	gnss_test.startsvr();
	rtklib::traceclose();
	return 0;
}