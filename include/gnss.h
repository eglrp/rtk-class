#ifndef GNSS_H_
#define GNSS_H_
#include <string>
#include "rtklib.h"


extern "C"  __declspec(dllimport) rtklib::opt_t sysopts[];
extern "C"  __declspec(dllimport) rtklib::solopt_t solopt_default;
static int strtype[] = {                  /* stream types */
	STR_SERIAL, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE
};
static int strfmt[] = {                   /* stream formats */
	STRFMT_UBX, STRFMT_RTCM3, STRFMT_SP3, SOLF_LLH, SOLF_NMEA
};
#define TIMOPT  "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT  "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT  "0:off,1:std+2:age/ratio/ns"
#define ISTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,7:ntripcli,8:ftp,9:http"
#define OSTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr,11:ntripc_c"
#define FMTOPT  "0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,10:nvs,11:binex,12:rt17,13:sbf,14:cmr,17:sp3"
#define NMEOPT  "0:off,1:latlon,2:single"
#define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea,4:stat"
#define MSGOPT  "0:all,1:rover,2:base,3:corr"

#define MAXSTR      1024                /* max length of a stream */

class svropt
{
public:
	svropt();
	char strpath[8][1024]; /* stream paths */
	int svrcycle = 10;            /* server cycle (ms) */
	int timeout = 10000;         /* timeout time (ms) */
	int reconnect = 10000;         /* reconnect interval (ms) */
	int nmeacycle = 5000;          /* nmea request cycle (ms) */
	int buffsize = 32768;         /* input buffer size (bytes) */
	int navmsgsel = 0;             /* navigation mesaage select */
	char proxyaddr[256];          /* http/ntrip proxy */
	int nmeareq = 0;             /* nmea request type (0:off,1:lat/lon,2:single) */
	double nmeapos[3];       /* nmea position (lat/lon/height) (deg,m) */
	char rcvcmds[3][1024];    /* receiver commands files */
	char startcmd[1024];        /* start command */
	char stopcmd[1024];        /* stop command */
	int moniport = 0;             /* monitor port */
	int keepalive = 0;             /* keep alive flag */
	int fswapmargin = 30;            /* file swap margin (s) */
	char sta_name[256];
};

class GNSS
{
public:
	GNSS();
	GNSS(std::string f);	
	int initgnss();
	int startsvr();
	int stopsvr();
	int setconf(const std::string& f){ conf_file = f; };
	int getstatus();
	rtklib::sol_t getsol();
	int getpos(double (&pos_rr)[6]);
	int getdata(rtklib::obs_t &, int &, rtklib::nav_t &);
	void readant(rtklib::prcopt_t *, rtklib::nav_t *);
private:
	rtklib::rtksvr_t svr;
	std::string conf_file;
	rtklib::prcopt_t prcopt;          /* processing options */
	rtklib::solopt_t solopt[2];       /* solution options */
	rtklib::filopt_t filopt;          /* file options */
	int intflg = 0;             /* interrupt flag (2:shtdown) */

	int timetype = 0;             /* time format (0:gpst,1:utc,2:jst,3:tow) */
	int soltype = 0;             /* sol format (0:dms,1:deg,2:xyz,3:enu,4:pyl) */
	int solflag = 2;             /* sol flag (1:std+2:age/ratio/ns) */
	int pro_type = 0;
	svropt svropt_;
	int streamopt[8];
	int streamfmt[5];

};






#endif
