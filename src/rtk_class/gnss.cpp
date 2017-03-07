#include "gnss.h"

svropt::svropt()
{
	for (int i = 0; i < 8;++i)
		strcpy(strpath[i],"");
	strcpy(proxyaddr, "");
	nmeapos[0] = 0.0;
	nmeapos[1] = 0.0;
	nmeapos[2] = 0.0;
	for (int i = 0; i < 3; ++i)
		strcpy(rcvcmds[i], "");
	strcpy(startcmd, "");
	strcpy(stopcmd, "");
	strcpy(sta_name, "");
}

GNSS::GNSS()
{
	conf_file = "";
	memset(&prcopt,0,sizeof(prcopt));
	memset(solopt, 0, 2*sizeof(prcopt));
	memset(&filopt, 0, sizeof(prcopt));
	memset(streamopt, 0, 8 * sizeof(int));
	memset(streamfmt, 0, 5 * sizeof(int));
	//memset(&svr, 0, sizeof(svr));
	//svr.state = 0;
}
GNSS::GNSS(std::string f):conf_file(f)
{
	memset(&prcopt, 0, sizeof(prcopt));
	memset(solopt, 0, 2 * sizeof(prcopt));
	memset(&filopt, 0, sizeof(prcopt));
	memset(streamopt, 0, 8*sizeof(int));
	memset(streamfmt, 0, 5*sizeof(int));
	//memset(&svr, 0, sizeof(svr));

}

int GNSS::initgnss()
{
	rtklib::rtksvrinit(&svr);
	rtklib::resetsysopts();
	static rtklib::opt_t rcvopts[] = {
		{ "process-typr", 0, (void *)&pro_type, "0:real;1:post" },
		{ "console-timetype", 3, (void *)&timetype, TIMOPT },
		{ "console-soltype", 3, (void *)&soltype, CONOPT },
		{ "console-solflag", 0, (void *)&solflag, FLGOPT },

		{ "inpstr1-type", 3, (void *)&strtype[0], ISTOPT },
		{ "inpstr2-type", 3, (void *)&strtype[1], ISTOPT },
		{ "inpstr3-type", 3, (void *)&strtype[2], ISTOPT },
		{ "inpstr1-path", 2, (void *)(svropt_.strpath[0]), "" },
		{ "inpstr2-path", 2, (void *)(svropt_.strpath[1]), "" },
		{ "inpstr3-path", 2, (void *)(svropt_.strpath[2]), "" },
		{ "inpstr1-format", 3, (void *)&strfmt[0], FMTOPT },
		{ "inpstr2-format", 3, (void *)&strfmt[1], FMTOPT },
		{ "inpstr3-format", 3, (void *)&strfmt[2], FMTOPT },
		{ "inpstr2-nmeareq", 3, (void *)&(svropt_.nmeareq), NMEOPT },
		{ "inpstr2-nmealat", 1, (void *)&(svropt_.nmeapos[0]), "deg" },
		{ "inpstr2-nmealon", 1, (void *)&(svropt_.nmeapos[1]), "deg" },
		{ "inpstr2-nmeahgt", 1, (void *)&(svropt_.nmeapos[2]), "m" },
		{ "outstr1-type", 3, (void *)&strtype[3], OSTOPT },
		{ "outstr2-type", 3, (void *)&strtype[4], OSTOPT },
		{ "outstr1-path", 2, (void *)(svropt_.strpath[3]), "" },
		{ "outstr2-path", 2, (void *)(svropt_.strpath[4]), "" },
		{ "outstr1-format", 3, (void *)&strfmt[3], SOLOPT },
		{ "outstr2-format", 3, (void *)&strfmt[4], SOLOPT },
		{ "logstr1-type", 3, (void *)&strtype[5], OSTOPT },
		{ "logstr2-type", 3, (void *)&strtype[6], OSTOPT },
		{ "logstr3-type", 3, (void *)&strtype[7], OSTOPT },
		{ "logstr1-path", 2, (void *)(svropt_.strpath[5]), "" },
		{ "logstr2-path", 2, (void *)(svropt_.strpath[6]), "" },
		{ "logstr3-path", 2, (void *)(svropt_.strpath[7]), "" },

		{ "misc-svrcycle", 0, (void *)&(svropt_.svrcycle), "ms" },
		{ "misc-timeout", 0, (void *)&(svropt_.timeout), "ms" },
		{ "misc-reconnect", 0, (void *)&(svropt_.reconnect), "ms" },
		{ "misc-nmeacycle", 0, (void *)&(svropt_.nmeacycle), "ms" },
		{ "misc-buffsize", 0, (void *)&(svropt_.buffsize), "bytes" },
		{ "misc-navmsgsel", 3, (void *)&(svropt_.navmsgsel), MSGOPT },
		{ "misc-proxyaddr", 2, (void *)(svropt_.proxyaddr), "" },
		{ "misc-fswapmargin", 0, (void *)&(svropt_.fswapmargin), "s" },


		{ "file-cmdfile1", 2, (void *)(svropt_.rcvcmds[0]), "" },
		{ "file-cmdfile2", 2, (void *)(svropt_.rcvcmds[1]), "" },
		{ "file-cmdfile3", 2, (void *)(svropt_.rcvcmds[2]), "" },

		{ "", 0, NULL, "" }
	};
	//extern rtklib::opt_t rtklib::sysopts[];

	//int aaa = pppppp;
	if (!rtklib::loadopts(conf_file.c_str(), rcvopts) || !rtklib::loadopts(conf_file.c_str(), sysopts)) {
		fprintf(stderr, "no options file: %s. defaults used\n", conf_file.c_str());
	}
	rtklib::getsysopts(&prcopt, solopt, &filopt);
	return 0;
}

int GNSS::startsvr()
{

	static rtklib::sta_t sta[MAXRCV] = { { "" } };
	static char strpath[8][MAXSTR] = { "", "", "", "", "", "", "", "" }; /* stream paths */
	double pos[3], npos[3];
	char *ropts[] = { "", "", "" };
	char *paths[] = {
		svropt_.strpath[0],
		svropt_.strpath[1],
		svropt_.strpath[2],
		svropt_.strpath[3],
		svropt_.strpath[4],
		svropt_.strpath[5],
		svropt_.strpath[6],
		svropt_.strpath[7]
	};
	char errmsg[2048] = "";
	int i, ret;

	rtklib::trace(3, "startsvr:\n");

	if (prcopt.refpos == 4) { /* rtcm */
		for (i = 0; i < 3; i++) prcopt.rb[i] = 0.0;
	}
	pos[0] = svropt_.nmeapos[0] * D2R;
	pos[1] = svropt_.nmeapos[1] * D2R;
	pos[2] = svropt_.nmeapos[2];
	rtklib::pos2ecef(pos, npos);

	/* read antenna file */
	readant(&prcopt, &svr.nav);

	/* read dcb file */
	if (filopt.dcb) {
		strcpy(sta[0].name, svropt_.sta_name);
		rtklib::readdcb(filopt.dcb, &svr.nav, sta);
	}
	/* open geoid data file */
	if (solopt[0].geoid > 0 && !rtklib::opengeoid(solopt[0].geoid, filopt.geoid)) {
		rtklib::trace(2, "geoid data open error: %s\n", filopt.geoid);
		//vt_printf(vt, "geoid data open error: %s\n", filopt.geoid);
	}

	/* set stream options */
	streamopt[0] = svropt_.timeout;
	streamopt[1] = svropt_.reconnect;
	streamopt[2] = 1000;
	streamopt[3] = svropt_.buffsize;
	streamopt[4] = svropt_.fswapmargin;
	rtklib::strsetopt(streamopt);

	if (strfmt[2] == 8) strfmt[2] = STRFMT_SP3;

	/* set ftp/http directory and proxy */
	rtklib::strsetdir(filopt.tempdir);
	rtklib::strsetproxy(svropt_.proxyaddr);

	/* execute start command */
	//if (*startcmd && (ret = system(startcmd))) {
	//	trace(2, "command exec error: %s (%d)\n", startcmd, ret);
	//vt_printf(vt, "command exec error: %s (%d)\n", startcmd, ret);
	//}
	solopt[0].posf = strfmt[3];
	solopt[1].posf = strfmt[4];

	/* start rtk server */
	if (!rtklib::rtksvrstart(&svr, svropt_.svrcycle, svropt_.buffsize, strtype, paths, strfmt, svropt_.navmsgsel,
		NULL, NULL, ropts, svropt_.nmeacycle, svropt_.nmeareq, npos, &prcopt,
		solopt, NULL, errmsg)) {
		rtklib::trace(2, "rtk server start error (%s)\n", errmsg);
		//vt_printf(vt, "rtk server start error (%s)\n", errmsg);
		return 0;
	}
	return 1;
}
int GNSS::stopsvr()
{
	if (!svr.state) return 0;
	/* stop rtk server */
	rtklib::rtksvrstop(&svr, NULL);
	return 1;
}
int GNSS::getstatus()
{
	rtklib::rtksvrlock(&svr);
	unsigned char buff[1024];
	if (svr.rtk.sol.stat != SOLQ_NONE)
	{
		rtklib::outsols(buff, &(svr.rtk.sol), svr.rtk.rb, &solopt_default);
		printf("%s", buff);
	}
	else
		printf("There is no solution yet, please wait a moment.\n");
	
	rtklib::rtksvrunlock(&svr);
	return 0;
}
int GNSS::getpos(double(&pos_rr)[6])
{
	rtklib::rtksvrlock(&svr);
	for (int i = 0; i < 6;++i)
		pos_rr[i]=svr.rtk.sol.rr[i];
	rtklib::rtksvrunlock(&svr);
	return 0;
}

int GNSS::getdata(rtklib::obs_t &obs, int &n, rtklib::nav_t &nav)
{
	rtklib::obsd_t data[MAXOBS * 2];
	obs = { 0 };
	obs.data = data;
	rtklib::rtksvrlock(&svr);
	n = 0;
	//obs.n = 0;
	for (int j = 0; j < svr.obs[0][0].n&&obs.n < MAXOBS * 2; j++) {
		obs.data[obs.n++] = svr.obs[0][0].data[j];
	}
	for (int j = 0; j < svr.obs[1][0].n&&obs.n < MAXOBS * 2; j++) {
		obs.data[obs.n++] = svr.obs[1][0].data[j];
	}
	n = obs.n;
	nav = svr.nav;
	rtklib::rtksvrunlock(&svr);
	return 0;
}

rtklib::sol_t GNSS::getsol()
{
	rtklib::rtksvrlock(&svr);
	rtklib::sol_t ret = svr.rtk.sol;
	rtklib::rtksvrunlock(&svr);
	return ret;
}
void GNSS::readant(rtklib::prcopt_t *opt, rtklib::nav_t *nav)
{
	const rtklib::pcv_t pcv0 = { 0 };
	rtklib::pcvs_t pcvr = { 0 }, pcvs = { 0 };
	rtklib::pcv_t *pcv;
	rtklib::gtime_t time = rtklib::timeget();
	int i;

	rtklib::trace(3, "readant:\n");

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
	//return 0;
}