/*------------------------------------------------------------------------------
* str2str.c : console version of stream server
*
*          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:54:53 $
* history : 2009/06/17  1.0  new
*           2011/05/29  1.1  add -f, -l and -x option
*           2011/11/29  1.2  fix bug on recognize ntrips:// (rtklib_2.4.1_p4)
*           2012/12/25  1.3  add format conversion functions
*                            add -msg, -opt and -sta options
*                            modify -p option
*           2013/01/25  1.4  fix bug on showing message
*           2014/02/21  1.5  ignore SIG_HUP
*           2014/08/10  1.5  fix bug on showing message
*           2014/08/26  1.6  support input format gw10, binex and rt17
*           2014/10/14  1.7  use stdin or stdout if option -in or -out omitted
*           2014/11/08  1.8  add option -a, -i and -o
*           2015/03/23  1.9  fix bug on parsing of command line options
*           2016/01/23  1.10 enable septentrio
*           2016/01/26  1.11 fix bug on station position by -p option (#126)
*                            add option -px
*           2016/07/01  1.12 support CMR/CMR+
*           2016/07/23  1.13 add option -c1 -c2 -c3 -c4
*           2016/09/03  1.14 support ntrip caster
*                            add option -ft,-fl
*           2016/09/06  1.15 add reload soure table by USR2 signal
*           2016/09/17  1.16 add option -b
*-----------------------------------------------------------------------------*/
#include <signal.h>
#include <stdio.h>
#include <conio.h>
//#include <unistd.h>
#include "rtklib.h"

using namespace rtklib;
#define PRGNAME     "rtkrcv"            /* program name */
#define CMDPROMPT   "rtkrcv> "          /* command prompt */
#define MAXCON      32                  /* max number of consoles */
#define MAXARG      10                  /* max number of args in a command */
#define MAXCMD      256                 /* max length of a command */
#define MAXSTR      1024                /* max length of a stream */
#define OPTSDIR     "."                 /* default config directory */
#define OPTSFILE    "rtkrcv.conf"       /* default config file */
#define NAVIFILE    "rtkrcv.nav"        /* navigation save file */
#define STATFILE    "rtkrcv_%Y%m%d%h%M.stat"  /* solution status file */
#define TRACEFILE   "rtkrcv_%Y%m%d%h%M.trace" /* debug trace file */
#define INTKEEPALIVE 1000               /* keep alive interval (ms) */
#define MAXFILE     8                   /* max number of input files */

static rtksvr_t svr;                    /* rtk server struct */
static stream_t moni;                   /* monitor stream */

static int intflg = 0;             /* interrupt flag (2:shtdown) */

static char passwd[MAXSTR] = "admin";     /* login password */
static int timetype = 0;             /* time format (0:gpst,1:utc,2:jst,3:tow) */
static int soltype = 0;             /* sol format (0:dms,1:deg,2:xyz,3:enu,4:pyl) */
static int solflag = 2;             /* sol flag (1:std+2:age/ratio/ns) */
static int strtype[] = {                  /* stream types */
	STR_SERIAL, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE
};
static char strpath[8][MAXSTR] = { "", "", "", "", "", "", "", "" }; /* stream paths */
static int strfmt[] = {                   /* stream formats */
	STRFMT_UBX, STRFMT_RTCM3, STRFMT_SP3, SOLF_LLH, SOLF_NMEA
};
static int svrcycle = 10;            /* server cycle (ms) */
static int timeout = 10000;         /* timeout time (ms) */
static int reconnect = 10000;         /* reconnect interval (ms) */
static int nmeacycle = 5000;          /* nmea request cycle (ms) */
static int buffsize = 32768;         /* input buffer size (bytes) */
static int navmsgsel = 0;             /* navigation mesaage select */
static char proxyaddr[256] = "";          /* http/ntrip proxy */
static int nmeareq = 0;             /* nmea request type (0:off,1:lat/lon,2:single) */
static double nmeapos[] = { 0, 0, 0 };       /* nmea position (lat/lon/height) (deg,m) */
static char rcvcmds[3][MAXSTR] = { "" };    /* receiver commands files */
static char startcmd[MAXSTR] = "";        /* start command */
static char stopcmd[MAXSTR] = "";        /* stop command */
static int modflgr[256] = { 0 };           /* modified flags of receiver options */
static int modflgs[256] = { 0 };           /* modified flags of system options */
static int moniport = 0;             /* monitor port */
static int keepalive = 0;             /* keep alive flag */
static int fswapmargin = 30;            /* file swap margin (s) */
static char sta_name[256] = "";           /* station name */

static prcopt_t prcopt;                 /* processing options */
static solopt_t solopt[2] = { { 0 } };        /* solution options */
static filopt_t filopt = { "" };          /* file options */
#define ESC_CLEAR   "\033[H\033[2J"     /* ansi/vt100 escape: erase screen */
#define ESC_RESET   "\033[0m"           /* ansi/vt100: reset attribute */
#define ESC_BOLD    "\033[1m"           /* ansi/vt100: bold */

#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))
static const char rcsid[] = "$Id:$";

//#define PRGNAME     "str2str"          /* program name */
//#define MAXSTR      5                  /* max number of streams */
//#define TRFILE      "str2str.trace"    /* trace file */

/* global variables ----------------------------------------------------------*/
static strsvr_t strsvr;                /* stream server */
static volatile int intrflg = 0;         /* interrupt flag */
static char srctbl[1024] = "";           /* source table file */

typedef struct {                       /* console type */
	int state;                         /* state (0:stop,1:run) */
	//vt_t *vt;                          /* virtual terminal */
	thread_t thread;                  /* console thread */
} con_t;

/* help text -----------------------------------------------------------------*/
static const char *help[] = {
	"",
	" usage: str2str [-in stream] [-out stream [-out stream...]] [options]",
	"",
	" Input data from a stream and divide and output them to multiple streams",
	" The input stream can be serial, tcp client, tcp server, ntrip client, or",
	" file. The output stream can be serial, tcp client, tcp server, ntrip server,",
	" or file. str2str is a resident type application. To stop it, type ctr-c in",
	" console if run foreground or send signal SIGINT for background process.",
	" if run foreground or send signal SIGINT for background process.",
	" if both of the input stream and the output stream follow #format, the",
	" format of input messages are converted to output. To specify the output",
	" messages, use -msg option. If the option -in or -out omitted, stdin for",
	" input or stdout for output is used. If the stream in the option -in or -out",
	" is null, stdin or stdout is used as well. To reload ntrip source table",
	" specified by the option -ft, send SIGUSR2 to the process",
	" Command options are as follows.",
	"",
	" -in  stream[#format] input  stream path and format",
	" -out stream[#format] output stream path and format",
	"",
	"  stream path",
	"    serial       : serial://port[:brate[:bsize[:parity[:stopb[:fctr]]]]]",
	"    tcp server   : tcpsvr://:port",
	"    tcp client   : tcpcli://addr[:port]",
	"    ntrip client : ntrip://[user[:passwd]@]addr[:port][/mntpnt]",
	"    ntrip server : ntrips://[:passwd@]addr[:port]/mntpnt[:str] (only out)",
	"    ntrip caster server: ntripc_s://[:passwd@][:port] (only in)",
	"    ntrip caster client: ntripc_c://[user:passwd@][:port]/mntpnt (only out)",
	"    file         : [file://]path[::T][::+start][::xseppd][::S=swap]",
	"",
	"  format",
	"    rtcm2        : RTCM 2 (only in)",
	"    rtcm3        : RTCM 3",
	"    nov          : NovAtel OEMV/4/6,OEMStar (only in)",
	"    oem3         : NovAtel OEM3 (only in)",
	"    ubx          : ublox LEA-4T/5T/6T (only in)",
	"    ss2          : NovAtel Superstar II (only in)",
	"    hemis        : Hemisphere Eclipse/Crescent (only in)",
	"    stq          : SkyTraq S1315F (only in)",
	"    gw10         : Furuno GW10 (only in)",
	"    javad        : Javad (only in)",
	"    nvs          : NVS BINR (only in)",
	"    binex        : BINEX (only in)",
	"    rt17         : Trimble RT17 (only in)",
	"    sbf          : Septentrio SBF (only in)",
	"    cmr          : CMR/CMR+ (only in)",
	"",
	" -msg \"type[(tint)][,type[(tint)]...]\"",
	"                   rtcm message types and output intervals (s)",
	" -sta sta          station id",
	" -opt opt          receiver dependent options",
	" -s  msec          timeout time (ms) [10000]",
	" -r  msec          reconnect interval (ms) [10000]",
	" -n  msec          nmea request cycle (m) [0]",
	" -f  sec           file swap margin (s) [30]",
	" -c  file          input commands file [no]",
	" -c1 file          output 1 commands file [no]",
	" -c2 file          output 2 commands file [no]",
	" -c3 file          output 3 commands file [no]",
	" -c4 file          output 4 commands file [no]",
	" -p  lat lon hgt   station position (latitude/longitude/height) (deg,m)",
	" -px x y z         station position (x/y/z-ecef) (m)",
	" -a  antinfo       antenna info (separated by ,)",
	" -i  rcvinfo       receiver info (separated by ,)",
	" -o  e n u         antenna offset (e,n,u) (m)",
	" -l  local_dir     ftp/http local directory []",
	" -x  proxy_addr    http/ntrip proxy address [no]",
	" -b  str_no        relay back messages from output str to input str [no]",
	" -t  level         trace level [0]",
	" -ft file          ntrip souce table file []",
	" -fl file          log file [str2str.trace]",
	" -h                print help",
};
/* print help ----------------------------------------------------------------*/
static void printhelp(void)
{
	int i;
	for (i = 0; i < sizeof(help) / sizeof(*help); i++) fprintf(stderr, "%s\n", help[i]);
	exit(0);
}
/* signal handler ------------------------------------------------------------*/
static void sigfunc(int sig)
{
	intrflg = 1;
}
/* reload source table by SIGUSR2 --------------------------------------------*/
static void reload_srctbl(int sig)
{
	strsvrsetsrctbl(&strsvr, srctbl);
	//signal(SIGUSR2,reload_srctbl);
}
/* decode format -------------------------------------------------------------*/
static void decodefmt(char *path, int *fmt)
{
	char *p;

	*fmt = -1;

	if ((p = strrchr(path, '#'))) {
		if (!strcmp(p, "#rtcm2")) *fmt = STRFMT_RTCM2;
		else if (!strcmp(p, "#rtcm3")) *fmt = STRFMT_RTCM3;
		else if (!strcmp(p, "#nov")) *fmt = STRFMT_OEM4;
		else if (!strcmp(p, "#oem3")) *fmt = STRFMT_OEM3;
		else if (!strcmp(p, "#ubx")) *fmt = STRFMT_UBX;
		else if (!strcmp(p, "#ss2")) *fmt = STRFMT_SS2;
		else if (!strcmp(p, "#hemis")) *fmt = STRFMT_CRES;
		else if (!strcmp(p, "#stq")) *fmt = STRFMT_STQ;
		else if (!strcmp(p, "#gw10")) *fmt = STRFMT_GW10;
		else if (!strcmp(p, "#javad")) *fmt = STRFMT_JAVAD;
		else if (!strcmp(p, "#nvs")) *fmt = STRFMT_NVS;
		else if (!strcmp(p, "#binex")) *fmt = STRFMT_BINEX;
		else if (!strcmp(p, "#rt17")) *fmt = STRFMT_RT17;
		else if (!strcmp(p, "#sbf")) *fmt = STRFMT_SEPT;
		else if (!strcmp(p, "#cmr")) *fmt = STRFMT_CMR;
		else return;
		*p = '\0';
	}
}
/* decode stream path --------------------------------------------------------*/
static int decodepath(const char *path, int *type, char *strpath, int *fmt)
{
	char buff[1024], *p;

	strcpy(buff, path);

	/* decode format */
	decodefmt(buff, fmt);

	/* decode type */
	if (!(p = strstr(buff, "://"))) {
		strcpy(strpath, buff);
		*type = STR_FILE;
		return 1;
	}
	if (!strncmp(path, "serial", 6)) *type = STR_SERIAL;
	else if (!strncmp(path, "tcpsvr", 6)) *type = STR_TCPSVR;
	else if (!strncmp(path, "tcpcli", 6)) *type = STR_TCPCLI;
	else if (!strncmp(path, "ntripc_s", 8)) *type = STR_NTRIPC_S;
	else if (!strncmp(path, "ntripc_c", 8)) *type = STR_NTRIPC_C;
	else if (!strncmp(path, "ntrips", 6)) *type = STR_NTRIPSVR;
	else if (!strncmp(path, "ntrip", 5)) *type = STR_NTRIPCLI;
	else if (!strncmp(path, "file", 4)) *type = STR_FILE;
	else {
		fprintf(stderr, "stream path error: %s\n", buff);
		return 0;
	}
	strcpy(strpath, p + 3);
	return 1;
}
/* read receiver commands ----------------------------------------------------*/
static void readcmd(const char *file, char *cmd, int type)
{
	FILE *fp;
	char buff[MAXSTR], *p = cmd;
	int i = 0;

	*p = '\0';

	if (!(fp = fopen(file, "r"))) return;

	while (fgets(buff, sizeof(buff), fp)) {
		if (*buff == '@') i++;
		else if (i == type&&p + strlen(buff) + 1 < cmd + MAXRCVCMD) {
			p += sprintf(p, "%s", buff);
		}
	}
	fclose(fp);
}
static const char *usage[] = {
	"usage: rtkrcv [-s][-p port][-d dev][-o file][-w pwd][-r level][-t level][-sta sta]",
	"options",
	"  -s         start RTK server on program startup",
	"  -p port    port number for telnet console",
	"  -m port    port number for monitor stream",
	"  -d dev     terminal device for console",
	"  -o file    processing options file",
	"  -w pwd     login password for remote console (\"\": no password)",
	"  -r level   output solution status file (0:off,1:states,2:residuals)",
	"  -t level   debug trace level (0:off,1-5:on)",
	"  -sta sta   station name for receiver dcb"
};
static const char *helptxt[] = {
	"start                 : start rtk server",
	"stop                  : stop rtk server",
	"restart               : restart rtk sever",
	"solution [cycle]      : show solution",
	"status [cycle]        : show rtk status",
	"satellite [-n] [cycle]: show satellite status",
	"observ [-n] [cycle]   : show observation data",
	"navidata [cycle]      : show navigation data",
	"stream [cycle]        : show stream status",
	"error                 : show error/warning messages",
	"option [opt]          : show option(s)",
	"set opt [val]         : set option",
	"load [file]           : load options from file",
	"save [file]           : save options to file",
	"log [file|off]        : start/stop log to file",
	"help|? [path]         : print help",
	"exit|ctr-D            : logout console (only for telnet)",
	"shutdown              : shutdown rtk server",
	"!command [arg...]     : execute command in shell",
	""
};
static const char *pathopts[] = {         /* path options help */
	"stream path formats",
	"serial   : port[:bit_rate[:byte[:parity(n|o|e)[:stopb[:fctr(off|on)]]]]]",
	"file     : path[::T[::+offset][::xspeed]]",
	"tcpsvr   : :port",
	"tcpcli   : addr:port",
	"ntripsvr : user:passwd@addr:port/mntpnt[:str]",
	"ntripcli : user:passwd@addr:port/mntpnt",
	"ntripc_s : :passwd@:port",
	"ntripc_c : user:passwd@:port",
	"ftp      : user:passwd@addr/path[::T=poff,tint,off,rint]",
	"http     : addr/path[::T=poff,tint,off,rint]",
	""
};
/* receiver options table ----------------------------------------------------*/
#define TIMOPT  "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT  "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT  "0:off,1:std+2:age/ratio/ns"
#define ISTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,7:ntripcli,8:ftp,9:http"
#define OSTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr,11:ntripc_c"
#define FMTOPT  "0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,10:nvs,11:binex,12:rt17,13:sbf,14:cmr,17:sp3"
#define NMEOPT  "0:off,1:latlon,2:single"
#define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea,4:stat"
#define MSGOPT  "0:all,1:rover,2:base,3:corr"
static opt_t rcvopts[] = {
	{ "console-passwd", 2, (void *)passwd, "" },
	{ "console-timetype", 3, (void *)&timetype, TIMOPT },
	{ "console-soltype", 3, (void *)&soltype, CONOPT },
	{ "console-solflag", 0, (void *)&solflag, FLGOPT },

	{ "inpstr1-type", 3, (void *)&strtype[0], ISTOPT },
	{ "inpstr2-type", 3, (void *)&strtype[1], ISTOPT },
	{ "inpstr3-type", 3, (void *)&strtype[2], ISTOPT },
	{ "inpstr1-path", 2, (void *)strpath[0], "" },
	{ "inpstr2-path", 2, (void *)strpath[1], "" },
	{ "inpstr3-path", 2, (void *)strpath[2], "" },
	{ "inpstr1-format", 3, (void *)&strfmt[0], FMTOPT },
	{ "inpstr2-format", 3, (void *)&strfmt[1], FMTOPT },
	{ "inpstr3-format", 3, (void *)&strfmt[2], FMTOPT },
	{ "inpstr2-nmeareq", 3, (void *)&nmeareq, NMEOPT },
	{ "inpstr2-nmealat", 1, (void *)&nmeapos[0], "deg" },
	{ "inpstr2-nmealon", 1, (void *)&nmeapos[1], "deg" },
	{ "inpstr2-nmeahgt", 1, (void *)&nmeapos[2], "m" },
	{ "outstr1-type", 3, (void *)&strtype[3], OSTOPT },
	{ "outstr2-type", 3, (void *)&strtype[4], OSTOPT },
	{ "outstr1-path", 2, (void *)strpath[3], "" },
	{ "outstr2-path", 2, (void *)strpath[4], "" },
	{ "outstr1-format", 3, (void *)&strfmt[3], SOLOPT },
	{ "outstr2-format", 3, (void *)&strfmt[4], SOLOPT },
	{ "logstr1-type", 3, (void *)&strtype[5], OSTOPT },
	{ "logstr2-type", 3, (void *)&strtype[6], OSTOPT },
	{ "logstr3-type", 3, (void *)&strtype[7], OSTOPT },
	{ "logstr1-path", 2, (void *)strpath[5], "" },
	{ "logstr2-path", 2, (void *)strpath[6], "" },
	{ "logstr3-path", 2, (void *)strpath[7], "" },

	{ "misc-svrcycle", 0, (void *)&svrcycle, "ms" },
	{ "misc-timeout", 0, (void *)&timeout, "ms" },
	{ "misc-reconnect", 0, (void *)&reconnect, "ms" },
	{ "misc-nmeacycle", 0, (void *)&nmeacycle, "ms" },
	{ "misc-buffsize", 0, (void *)&buffsize, "bytes" },
	{ "misc-navmsgsel", 3, (void *)&navmsgsel, MSGOPT },
	{ "misc-proxyaddr", 2, (void *)proxyaddr, "" },
	{ "misc-fswapmargin", 0, (void *)&fswapmargin, "s" },

	{ "misc-startcmd", 2, (void *)startcmd, "" },
	{ "misc-stopcmd", 2, (void *)stopcmd, "" },

	{ "file-cmdfile1", 2, (void *)rcvcmds[0], "" },
	{ "file-cmdfile2", 2, (void *)rcvcmds[1], "" },
	{ "file-cmdfile3", 2, (void *)rcvcmds[2], "" },

	{ "", 0, NULL, "" }
};
/* signal handler ------------------------------------------------------------*/
//static void sigfunc(int sig)
//{
//	intrflg = 1;
//}

static void printusage(void)
{
	int i;
	for (i = 0; i < (int)(sizeof(usage) / sizeof(*usage)); i++) {
		fprintf(stderr, "%s\n", usage[i]);
	}
	exit(0);
}

static void readant(prcopt_t *opt, nav_t *nav)
{
	const pcv_t pcv0 = { 0 };
	pcvs_t pcvr = { 0 }, pcvs = { 0 };
	pcv_t *pcv;
	gtime_t time = timeget();
	int i;

	trace(3, "readant:\n");

	opt->pcvr[0] = opt->pcvr[1] = pcv0;
	if (!*filopt.rcvantp) return;

	if (readpcv(filopt.rcvantp, &pcvr)) {
		for (i = 0; i < 2; i++) {
			if (!*opt->anttype[i]) continue;
			if (!(pcv = searchpcv(0, opt->anttype[i], time, &pcvr))) {
				continue;
			}
			opt->pcvr[i] = *pcv;
		}
	}

	if (readpcv(filopt.satantp, &pcvs)) {
		for (i = 0; i < MAXSAT; i++) {
			if (!(pcv = searchpcv(i + 1, "", time, &pcvs))) continue;
			nav->pcvs[i] = *pcv;
		}
	}

	free(pcvr.pcv); free(pcvs.pcv);
}
static int startsvr()
{
	static sta_t sta[MAXRCV] = { { "" } };
	double pos[3], npos[3];
	char s1[3][MAXRCVCMD] = { "", "", "" }, *cmds[] = { NULL, NULL, NULL };
	char s2[3][MAXRCVCMD] = { "", "", "" }, *cmds_periodic[] = { NULL, NULL, NULL };
	char *ropts[] = { "", "", "" };
	char *paths[] = {
		strpath[0], strpath[1], strpath[2], strpath[3], strpath[4], strpath[5],
		strpath[6], strpath[7]
	};
	char errmsg[2048] = "";
	int i, ret, stropt[8] = { 0 };

	trace(3, "startsvr:\n");

	if (prcopt.refpos == 4) { /* rtcm */
		for (i = 0; i < 3; i++) prcopt.rb[i] = 0.0;
	}
	pos[0] = nmeapos[0] * D2R;
	pos[1] = nmeapos[1] * D2R;
	pos[2] = nmeapos[2];
	pos2ecef(pos, npos);

	/* read antenna file */
	readant(&prcopt, &svr.nav);

	/* read dcb file */
	if (filopt.dcb) {
		strcpy(sta[0].name, sta_name);
		readdcb(filopt.dcb, &svr.nav, sta);
	}
	/* open geoid data file */
	if (solopt[0].geoid > 0 && !opengeoid(solopt[0].geoid, filopt.geoid)) {
		trace(2, "geoid data open error: %s\n", filopt.geoid);
		//vt_printf(vt, "geoid data open error: %s\n", filopt.geoid);
	}
	for (i = 0; *rcvopts[i].name; i++) modflgr[i] = 0;
	for (i = 0; *sysopts[i].name; i++) modflgs[i] = 0;

	/* set stream options */
	stropt[0] = timeout;
	stropt[1] = reconnect;
	stropt[2] = 1000;
	stropt[3] = buffsize;
	stropt[4] = fswapmargin;
	strsetopt(stropt);

	if (strfmt[2] == 8) strfmt[2] = STRFMT_SP3;

	/* set ftp/http directory and proxy */
	strsetdir(filopt.tempdir);
	strsetproxy(proxyaddr);

	/* execute start command */
	//if (*startcmd && (ret = system(startcmd))) {
	//	trace(2, "command exec error: %s (%d)\n", startcmd, ret);
		//vt_printf(vt, "command exec error: %s (%d)\n", startcmd, ret);
	//}
	solopt[0].posf = strfmt[3];
	solopt[1].posf = strfmt[4];

	/* start rtk server */
	if (!rtksvrstart(&svr, svrcycle, buffsize, strtype, paths, strfmt, navmsgsel,
		cmds, cmds_periodic, ropts, nmeacycle, nmeareq, npos, &prcopt,
		solopt, NULL, errmsg)) {
		trace(2, "rtk server start error (%s)\n", errmsg);
		//vt_printf(vt, "rtk server start error (%s)\n", errmsg);
		return 0;
	}
	return 1;
}

/* stop rtk server -----------------------------------------------------------*/
static void stopsvr()
{
	char s[3][MAXRCVCMD] = { "", "", "" }, *cmds[] = { NULL, NULL, NULL };
	int i, ret;

	trace(3, "stopsvr:\n");

	if (!svr.state) return;

	/* stop rtk server */
	rtksvrstop(&svr, cmds);
}

int main(int argc, char **argv)
{
	con_t *con[MAXCON] = { 0 };
	int i,j,n, pro_model = 0, port = 0, outstat = 0, trace = 0, sock = 0,ret;
	char *dev = "", file[MAXSTR] = "";
	gtime_t ts = { 0 }, te = { 0 };
	double tint = 0.0, es[] = { 2000, 1, 1, 0, 0, 0 }, ee[] = { 2000, 12, 31, 23, 59, 59 }, pos[3];
	char *infile[MAXFILE], *outfile = "";


	argc = 1;
	argv[argc++] = "-real";
	argv[argc++] = "-o";
	argv[argc++] = "rtk.conf";
	//rover
	argv[argc++] = "D:\\OE10\\novatel_TRUENET\\data\\trimble_20160614.obs";
	//base
	argv[argc++] = "D:\\OE10\\novatel_TRUENET\\data\\TRUECORS_VRS_20160614.obs";
	argv[argc++] = "D:\\OE10\\novatel_TRUENET\\data\\brdm1660.16p";

	for (i = 1,n = 0; i < argc; i++) {
		if (!strcmp(argv[i], "-real")) pro_model = 0;
		else if (!strcmp(argv[i], "-post")) pro_model = 1;
		else if (!strcmp(argv[i], "-o") && i + 1 < argc)
		{
			strcpy(file, argv[++i]);
			resetsysopts();
			if (!loadopts(file, rcvopts) || !loadopts(file, sysopts)) {
				fprintf(stderr, "no options file: %s. defaults used\n", file);
			}
			getsysopts(&prcopt, solopt, &filopt);
		}
		else if (!strcmp(argv[i], "-ts") && i + 2 < argc) {
			sscanf(argv[++i], "%lf/%lf/%lf", es, es + 1, es + 2);
			sscanf(argv[++i], "%lf:%lf:%lf", es + 3, es + 4, es + 5);
			ts = epoch2time(es);
		}
		else if (!strcmp(argv[i], "-te") && i + 2 < argc) {
			sscanf(argv[++i], "%lf/%lf/%lf", ee, ee + 1, ee + 2);
			sscanf(argv[++i], "%lf:%lf:%lf", ee + 3, ee + 4, ee + 5);
			te = epoch2time(ee);
		}
		else if (!strcmp(argv[i], "-stat") && i + 1 < argc) outstat = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-trace") && i + 1 < argc) trace = atoi(argv[++i]);
		else if (n<MAXFILE) infile[n++] = argv[i];


		else if (!strcmp(argv[i], "-ti") && i + 1 < argc) tint = atof(argv[++i]);
		else if (!strcmp(argv[i], "-k") && i + 1 < argc) { ++i; continue; }
		else if (!strcmp(argv[i], "-p") && i + 1 < argc) 	prcopt.mode = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-f") && i + 1 < argc) prcopt.nf = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-m") && i + 1 < argc) prcopt.elmin = atof(argv[++i])*D2R;
		else if (!strcmp(argv[i], "-v") && i + 1 < argc) prcopt.thresar[0] = atof(argv[++i]);
		else if (!strcmp(argv[i], "-s") && i + 1 < argc) strcpy(solopt[0].sep, argv[++i]);
		else if (!strcmp(argv[i], "-sys") && i + 1 < argc) {
			if (!strcmp(argv[i + 1], "G")){ prcopt.navsys = SYS_GPS; i++; }
			else if (!strcmp(argv[i + 1], "C")){ prcopt.navsys = SYS_CMP; i++; }
			else if (!strcmp(argv[i + 1], "R")){ prcopt.navsys = SYS_GLO; i++; }
			else if (!strcmp(argv[i + 1], "GC")){ prcopt.navsys = SYS_CMP | SYS_GPS; i++; }
			else if (!strcmp(argv[i + 1], "GCR")){ prcopt.navsys = SYS_CMP | SYS_GPS | SYS_GLO; i++; }

		}
		else if (!strcmp(argv[i], "-d") && i + 1 < argc) solopt[0].timeu = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-b")) prcopt.soltype = 1;
		else if (!strcmp(argv[i], "-c")) prcopt.soltype = 2;
		else if (!strcmp(argv[i], "-i")) prcopt.modear = 2;
		else if (!strcmp(argv[i], "-h")) prcopt.modear = 3;
		else if (!strcmp(argv[i], "-t")) solopt[0].timef = 1;
		else if (!strcmp(argv[i], "-u")) solopt[0].times = TIMES_UTC;
		else if (!strcmp(argv[i], "-e")) solopt[0].posf = SOLF_XYZ;
		else if (!strcmp(argv[i], "-a")) solopt[0].posf = SOLF_ENU;
		else if (!strcmp(argv[i], "-n")) solopt[0].posf = SOLF_NMEA;
		else if (!strcmp(argv[i], "-g")) solopt[0].degf = 1;
		else if (!strcmp(argv[i], "-r") && i + 3 < argc) {
			prcopt.refpos = 0;
			for (j = 0; j < 3; j++) prcopt.rb[j] = atof(argv[++i]);
		}
		else if (!strcmp(argv[i], "-l") && i + 3 < argc) {
			prcopt.refpos = 0;
			for (j = 0; j < 3; j++) pos[j] = atof(argv[++i]);
			for (j = 0; j < 2; j++) pos[j] *= D2R;
			pos2ecef(pos, prcopt.rb);
		}
		else if (!strcmp(argv[i], "-y") && i + 1 < argc) solopt[0].sstat = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-x") && i + 1<argc) solopt[0].trace = atoi(argv[++i]);
		else if (*argv[i] == '-')
			printhelp();
	}

	if (trace > 0) {
		traceopen(TRACEFILE);
		tracelevel(trace);
	}
	switch (pro_model)
	{
	case 0:
		rtksvrinit(&svr);

		/* load options file */
		if (!*file) sprintf(file, "%s/%s", OPTSDIR, OPTSFILE);



		/* read navigation data */
		if (!readnav(NAVIFILE, &svr.nav)) {
			fprintf(stderr, "no navigation data: %s\n", NAVIFILE);
		}
		if (outstat > 0) {
			rtkopenstat(STATFILE, outstat);
		}

		//signal(SIGTERM, SIG_IGN);
		//signal(SIGINT, SIG_IGN);
		//signal(SIGHUP ,SIG_IGN);
		//signal(SIGPIPE,SIG_IGN);

		/* start rtk server */
		startsvr();
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
			sleepms(100);
		}
		/* stop rtk server */
		stopsvr();
		printf("Exitting...");
		/* save navigation data */
		if (!savenav(NAVIFILE, &svr.nav)) {
			fprintf(stderr, "navigation data save error: %s\n", NAVIFILE);
		}

		break;
	case 1:
		if (n <= 0) {
			return -2;
		}
		//solopt.posf = SOLF_DXYZ;
		ret = postpos(ts, te, tint, 0.0, &prcopt, solopt, &filopt, infile, n, outfile, "", "");
	default:
		break;
	}

	/* initialize rtk server and monitor port */
	traceclose();
	return 0;
}
