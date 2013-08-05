/*
 * FTDI232R USB JTAG programmer
 *
 * v 1.02 2013/08/03
 *
 * (c) 2010 - 2013 Marko Zec <zec@fer.hr>
 *
 * This software is NOT freely redistributable, neither in source nor in
 * binary format.  Usage in binary format permitted exclusively for
 * programming FER's FPGA boards.
 *
 *
 * TODO:
 * 
 * - WIN32: check for USB device string description
 *
 * - Cryptographically bind USB serial number with FTDI chip ID
 *
 * - JTAG scan / identify chain on entry.
 *
 * - verify SRAM / FLASH
 *
 * - RUN TEST delay downscaling.
 *
 * - disable resetting the TAP on entry / leave?
 *
 * - execute SVF commands provided as command line args?
 */


#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#if !defined(__linux__) && !defined(WIN32)
#define USE_PPI
#endif

#ifdef WIN32
#include <windows.h>
#include <ftd2xx.h>
#else
#include <sys/time.h>
#ifdef USE_PPI
#include <dev/ppbus/ppi.h>
#include <dev/ppbus/ppbconf.h>
#endif
#include <ftdi.h>
#endif

#ifdef WIN32
#define	EOPNOTSUPP		-1000
#define	BITMODE_OFF		0x0
#define	BITMODE_BITBANG		0x1
#define	BITMODE_SYNCBB		0x4
#define	BITMODE_CBUS		0x20
#endif


/* Forward declarations */
static int commit(int);
static void set_state(int);
static int exec_svf_tokenized(int, char **);
static int send_dr(int, char *, char *, char *);
static int send_ir(int, char *, char *, char *);
static int exec_svf_mem(char *, int, int);


enum svf_cmd {
	SVF_SDR, SVF_SIR, SVF_STATE, SVF_RUNTEST, SVF_HDR, SVF_HIR,
	SVF_TDR, SVF_TIR, SVF_ENDDR, SVF_ENDIR, SVF_FREQUENCY, SVF_UNKNOWN
};

static struct svf_cmdtable {
	enum svf_cmd cmd_id;
	char *cmd_str;
} svf_cmdtable[] = {
	{SVF_SDR,		"SDR"},
	{SVF_SIR,		"SIR"},
	{SVF_STATE,		"STATE"},
	{SVF_RUNTEST,		"RUNTEST"},
	{SVF_HDR,		"HDR"},
	{SVF_HIR,		"HIR"},
	{SVF_TDR,		"TDR"},
	{SVF_TIR,		"TIR"},
	{SVF_ENDDR,		"ENDDR"},
	{SVF_ENDIR,		"ENDIR"},
	{SVF_FREQUENCY,		"FREQUENCY"},
	{SVF_UNKNOWN,		NULL}
};


enum tap_state {
	RESET, IDLE,
	DRSELECT, DRCAPTURE, DRSHIFT, DREXIT1, DRPAUSE, DREXIT2, DRUPDATE,
	IRSELECT, IRCAPTURE, IRSHIFT, IREXIT1, IRPAUSE, IREXIT2, IRUPDATE,
	UNDEFINED, UNSUPPORTED,
};

static struct tap_statetable {
	enum tap_state state_id;
	char *state_str;
} tap_statetable[] = {
	{RESET,			"RESET"},
	{IDLE,			"IDLE"},
	{DRSELECT,		"DRSELECT"},
	{DRCAPTURE,		"DRCAPTURE"},
	{DRSHIFT,		"DRSHIFT"},
	{DREXIT1,		"DREXIT1"},
	{DRPAUSE,		"DRPAUSE"},
	{DREXIT2,		"DREXIT2"},
	{DRUPDATE,		"DRUPDATE"},
	{IRSELECT,		"IRSELECT"},
	{IRCAPTURE,		"IRCAPTURE"},
	{IRSHIFT,		"IRSHIFT"},
	{IREXIT1,		"IREXIT1"},
	{IRPAUSE,		"IRPAUSE"},
	{IREXIT2,		"IREXIT2"},
	{IRUPDATE,		"IRUPDATE"},
	{UNDEFINED,		"UNDEFINED"},
	{UNSUPPORTED,		NULL}
};

#define	STATE2STR(state)	(tap_statetable[state].state_str)


static enum port_mode {
	PORT_MODE_ASYNC, PORT_MODE_SYNC, PORT_MODE_UART, PORT_MODE_UNKNOWN
} port_mode = PORT_MODE_UNKNOWN;


static enum cable_hw {
	CABLE_HW_USB, CABLE_HW_PPI, CABLE_HW_UNKNOWN
} cable_hw = CABLE_HW_UNKNOWN;


static struct cable_hw_map {
	int cable_hw;
	char *cable_path;
} cable_hw_map[] = {
	{CABLE_HW_USB,		"FER ULXP2 board JTAG / UART"},
	{CABLE_HW_USB,		"FER ULX2S board JTAG / UART"},
	{CABLE_HW_UNKNOWN,	NULL},
};


#define	USB_BAUDS		1000000

#define	USB_TCK			0x20
#define	USB_TMS			0x80
#define	USB_TDI			0x08
#define	USB_TDO			0x40
#define	USB_CBUS_LED		0x02

#define	PPI_TCK			0x02
#define	PPI_TMS			0x04
#define	PPI_TDI			0x01
#define	PPI_TDO			0x40

#define	USB_BUFLEN_ASYNC	8192
#ifdef WIN32
#define	USB_BUFLEN_SYNC		4096
#else
#define	USB_BUFLEN_SYNC		384
#endif

#define	BUFLEN_MAX		USB_BUFLEN_ASYNC /* max(SYNC, ASYNC) */

#define	LED_BLINK_RATE		250

static char *statc = "-\\|/";

/* Runtime globals */
static int cur_s = UNDEFINED;
static unsigned char txbuf[8 * BUFLEN_MAX];
static int txpos = 0;
static int need_led_blink;	/* Schedule CBUS led toggle */
static int last_ledblink_ms;	/* Last time we toggled the CBUS LED */
static int led_state;		/* CBUS LED indicator state */
static int blinker_phase = 0;
static int progress_perc = 0;
static int bauds = 115200;


#ifdef WIN32
static FT_HANDLE ftHandle;		/* USB port handle */
static int quick_mode = 1;
#else
static struct ftdi_context fc;	/* USB port handle */
#ifdef USE_PPI
static int ppi;			/* Parallel port handle */
#endif
#endif


/* ms_sleep() sleeps for at least the number of milliseconds given as arg */
#ifdef WIN32
#define	ms_sleep(delay_ms)	sleep(delay_ms);
#else
#define	ms_sleep(delay_ms)	usleep((delay_ms) * 1000);
#endif


static long
ms_uptime(void)
{
	long ms;
#ifndef WIN32
	struct timeval tv;

	gettimeofday(&tv, 0);
	ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
#else
	ms = GetTickCount();
#endif
	return (ms);
}


static int
set_port_mode(int mode)
{
	int res = 0;

	/* No-op if already in requested mode, or not using USB */
	if (!need_led_blink &&
	    (port_mode == mode || cable_hw != CABLE_HW_USB)) {
		port_mode = mode;
		return (0);
	}

	/* Flush any stale TX buffers */
	commit(1);

	/* Blink status LED by deactivating CBUS pulldown pin */
	if (need_led_blink) {
		need_led_blink = 0;
		led_state ^= USB_CBUS_LED;
		printf("\rProgramming: %d%% %c ",
		    progress_perc, statc[blinker_phase]);
		fflush(stdout);
		blinker_phase = (blinker_phase + 1) & 0x3;
	}

#ifdef WIN32
	if (mode == PORT_MODE_ASYNC && !quick_mode)
		mode = PORT_MODE_SYNC;
#endif

	switch (mode) {
	case PORT_MODE_SYNC:
#ifdef WIN32
		/*
		 * If switching to SYNC mode, attempt to allow for TX
		 * buffers to drain first.
		 */
		if (port_mode != PORT_MODE_SYNC)
			ms_sleep(10);

		res = FT_SetBitMode(ftHandle,
#else
		res = ftdi_set_bitmode(&fc,
#endif
		    USB_TCK | USB_TMS | USB_TDI | led_state,
		    BITMODE_SYNCBB | BITMODE_CBUS);

		if (port_mode == PORT_MODE_SYNC)
			break;

		/* Flush any stale RX buffers */
#ifdef WIN32
		for (res = 0; res < 2; res++) {
			do {} while (FT_StopInTask(ftHandle) != FT_OK);
			FT_Purge(ftHandle, FT_PURGE_RX);
			do {} while (FT_RestartInTask(ftHandle) != FT_OK);
		}
#else
		do {
			res = ftdi_read_data(&fc, &txbuf[0], sizeof(txbuf));
		} while (res == sizeof(txbuf));
#endif
		break;

	case PORT_MODE_ASYNC:
#ifdef WIN32
		res = FT_SetBitMode(ftHandle,
#else
		res = ftdi_set_bitmode(&fc,
#endif
		    USB_TCK | USB_TMS | USB_TDI | led_state,
		    BITMODE_BITBANG | BITMODE_CBUS);
		break;

	case PORT_MODE_UART:
#ifdef WIN32
		res = FT_SetBitMode(ftHandle, 0, BITMODE_OFF);
#else
		res = ftdi_disable_bitbang(&fc);
#endif
		break;

	default:
		res = -1;
	}

	port_mode = mode;
	return (res);
}


#ifdef WIN32
static char *
strtok_r(char *s1, const char *s2, char **lasts)
{
	char *ret;

	if (s1 == NULL)
		s1 = *lasts;
	while(*s1 && strchr(s2, *s1))
		++s1;
	if(*s1 == 0)
		return NULL;
	ret = s1;
	while(*s1 && !strchr(s2, *s1))
		++s1;
	if(*s1)
		*s1++ = 0;
	*lasts = s1;
	return (ret);
}


static int
setup_usb(void)
{
	FT_STATUS res;
	FT_DEVICE ftDevice;
	DWORD deviceID;
	char SerialNumber[16];
	char Description[64];

	res = FT_Open(0, &ftHandle);
	if (res != FT_OK) {
		fprintf(stderr, "FT_Open() failed\n");
		return (res);
	}

	res = FT_GetDeviceInfo(ftHandle, &ftDevice, &deviceID, SerialNumber,
	    Description, NULL);
	if (res != FT_OK) {
		fprintf(stderr, "FT_GetDeviceInfo() failed\n");
		return (res);
	}
	if (deviceID != 0x04036001) {
		fprintf(stderr,
		    "FT_GetDeviceInfo() found incompatible device\n");
		return (-1);
	}
	printf("Using USB cable: %s\n", Description);
    
	res = FT_SetBaudRate(ftHandle, USB_BAUDS);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetBaudRate() failed\n");
		return (res);
	}

#ifdef NOTYET
	FT_setUSB_Parameters();
	res = ftdi_write_data_set_chunksize(&fc, BUFLEN_MAX);
	if (res < 0) {
		fprintf(stderr, "ftdi_write_data_set_chunksize() failed\n");
		return (res);
	}
#endif

	res = FT_SetLatencyTimer(ftHandle, 1);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetLatencyTimer() failed\n");
		return (res);
	}

	res = FT_SetFlowControl(ftHandle, FT_FLOW_NONE, 0, 0);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetFlowControl() failed\n");
		return (res);
	}

	res = FT_SetBitMode(ftHandle, 0, BITMODE_BITBANG);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetBitMode() failed\n");
		return (res);
	}

	res = FT_SetTimeouts(ftHandle, 1000, 1000);
	if (res != FT_OK) {
		fprintf(stderr, "FT_SetTimeouts() failed\n");
		return (res);
	}

	FT_Purge(ftHandle, FT_PURGE_TX);
	FT_Purge(ftHandle, FT_PURGE_RX);

	return (0);
}


static int
shutdown_usb(void)
{

	int res;

	/* Pull TCK low so that we don't incidentally pulse it on next run. */
	memset(txbuf, 0, 100);
	FT_Write(ftHandle, txbuf, 100, (DWORD *) &res);
	if (res < 0) {
		fprintf(stderr, "FT_Write() failed\n");
		return (res);
	}

	/* Allow for the USB FIFO to drain, just in case. */
	ms_sleep(10);

	/* Clean up */
	res = set_port_mode(PORT_MODE_UART);
	if (res < 0) {
		fprintf(stderr, "set_port_mode() failed\n");
		return (res);
	}

	res = FT_SetLatencyTimer(ftHandle, 20);
	if (res < 0) {
		fprintf(stderr, "FT_SetLatencyTimer() failed\n");
		return (res);
	}

	res = FT_Close(ftHandle);
	if (res < 0) {
		fprintf(stderr, "FT_Close() failed\n");
		return (res);
	}

	return (0);
}
#endif /* WIN32 */


#ifndef WIN32
#ifdef USE_PPI
static int
setup_ppi(void)
{
	char c = 0;

	ppi = open("/dev/ppi0", O_RDWR);
	if (ppi < 0)
		return (errno);

	ioctl(ppi, PPISDATA, &c);
	ioctl(ppi, PPISSTATUS, &c);
	ioctl(ppi, PPIGSTATUS, &c);
	if ((c & 0xb6) != 0x06) {
		close (ppi);
		return (EINVAL);
	}

	return (0);
}


static void
shutdown_ppi(void)
{

	/* Pull TCK low so that we don't incidentally pulse it on next run. */
	txbuf[0] = 0;
	ioctl(ppi, PPISDATA, &txbuf[0]);

	close (ppi);
}
#endif


static int
setup_usb(void)
{
	int res;
	struct cable_hw_map *hmp;

	res = ftdi_init(&fc);
	if (res < 0) {
		fprintf(stderr, "ftdi_init() failed\n");
		return (res);
	}

	for (hmp = cable_hw_map; hmp->cable_hw != CABLE_HW_UNKNOWN; hmp++) {
		res = ftdi_usb_open_desc(&fc, 0x0403, 0x6001,
		    hmp->cable_path, NULL);
		if (res == 0)
			break;
	}
	if (res < 0)
		return (res);

	res = ftdi_set_baudrate(&fc, USB_BAUDS);
	if (res < 0) {
		fprintf(stderr, "ftdi_set_baudrate() failed\n");
		return (res);
	}

	res = ftdi_write_data_set_chunksize(&fc, BUFLEN_MAX);
	if (res < 0) {
		fprintf(stderr, "ftdi_write_data_set_chunksize() failed\n");
		return (res);
	}

	/* Reducing latency to 1 ms for BITMODE_SYNCBB is crucial! */
	res = ftdi_set_latency_timer(&fc, 1);
	if (res < 0) {
		fprintf(stderr, "ftdi_set_latency_timer() failed\n");
		return (res);
	}

	res = ftdi_set_bitmode(&fc, USB_TCK | USB_TMS | USB_TDI,
	    BITMODE_BITBANG);
	if (res < 0) {
		fprintf(stderr, "ftdi_set_bitmode() failed\n");
		return (EXIT_FAILURE);
	}

	return (0);
}


static int
shutdown_usb(void)
{
	int res;

	/* Pull TCK low so that we don't incidentally pulse it on next run. */
	txbuf[0] = 0;
	res = ftdi_write_data(&fc, &txbuf[0], 1);
	if (res < 0) {
		fprintf(stderr, "ftdi_write_data() failed\n");
		return (res);
	}

	/* Clean up */
	res = set_port_mode(PORT_MODE_UART);
	if (res < 0) {
		fprintf(stderr, "ftdi_disable_bitbang() failed\n");
		return (res);
	}

	res = ftdi_set_latency_timer(&fc, 20);
	if (res < 0) {
		fprintf(stderr, "ftdi_set_latency_timer() failed\n");
		return (res);
	}

	res = ftdi_usb_close(&fc);
	if (res < 0) {
		fprintf(stderr, "unable to close ftdi device: %d (%s)\n",
		    res, ftdi_get_error_string(&fc));
		return (res);
	}
	ftdi_deinit(&fc);

	return (0);
}
#endif /* !WIN32 */


static void
set_tms_tdi(int tms, int tdi)
{
	int val = 0;

	if (cable_hw == CABLE_HW_USB) {
		if (tms)
			val |= USB_TMS;
		if (tdi)
			val |= USB_TDI;
		txbuf[txpos++] = val;
		txbuf[txpos++] = val | USB_TCK;
	} else { /* PPI */
		if (tms)
			val |= PPI_TMS;
		if (tdi)
			val |= PPI_TDI;
		txbuf[txpos++] = val;
		txbuf[txpos++] = val | PPI_TCK;
	}

	if (txpos > sizeof(txbuf)) {
		fprintf(stderr, "txbuf overflow\n");
		if (cable_hw == CABLE_HW_USB)
			shutdown_usb();
		exit (EXIT_FAILURE);
	}
}


static int
send_generic(int bits, char *tdi, char *tdo, char *mask)
{
	int res, i, bitpos, tdomask, tdoval, maskval, val = 0;
	int rxpos, rxlen;

	if (cable_hw == CABLE_HW_USB)
		tdomask = USB_TDO;
	else
		tdomask = PPI_TDO;

	i = strlen(tdi);
	if (i != (bits + 3) / 4) {
		fprintf(stderr, "send_generic(): bitcount and tdi "
		    "data length do not match\n");
		return (EXIT_FAILURE);
	}
	if (tdo != NULL && strlen(tdo) != i) {
		if (mask != NULL && strlen(mask) != i) {
			fprintf(stderr, "send_generic(): tdi, tdo and mask "
			    "must be of same length\n");
			return (EXIT_FAILURE);
		}
		fprintf(stderr, "send_generic(): tdi and tdo "
		    "must be of same length\n");
		return (EXIT_FAILURE);
	}

	if (cur_s == DRPAUSE || cur_s == IRPAUSE ) {
		/* Move from *PAUSE to *EXIT2 state */
		set_tms_tdi(1, 0);
	}

	/* Move from *CAPTURE or *EXIT2 to *SHIFT state */
	set_tms_tdi(0, 0);

	/* Set up receive index / length */
	rxpos = txpos + 2;
	rxlen = bits;

	for (bitpos = 0; bits > 0; bits--) {
		if (bitpos == 0) {
			i--;
			val = tdi[i];
			if (val >= '0' && val <= '9')
				val = val - '0';
			else if (val >= 'A' && val <= 'F')
				val = val + 10 - 'A';
			else {
				fprintf(stderr, "send_generic():"
				    "TDI data not in hex format\n");
				return (EXIT_FAILURE);
			}
		}

		if (bits > 1)
			set_tms_tdi(0, val & 0x1);
		else
			set_tms_tdi(1, val & 0x1);

		val = val >> 1;
		bitpos = (bitpos + 1) & 0x3;
	}

	/* Move from *EXIT1 to *PAUSE state */
	set_tms_tdi(0, 0);

	/* Send / receive data on JTAG port */
	res = commit(0);

	/* Translate received bitstream into hex, apply mask, store in tdi */
	if (port_mode == PORT_MODE_SYNC) {
		if (mask != NULL)
			mask += strlen(tdi);
		if (tdo != NULL)
			tdo += strlen(tdi);
		tdi += strlen(tdi);
		val = 0;
		for (i = rxpos, bits = 0; bits < rxlen; i += 2) {
			val += (((txbuf[i] & tdomask) != 0) << (bits & 0x3));
			bits++;
			if ((bits & 0x3) == 0 || bits == rxlen) {
				if (mask != NULL) {
					/* Apply mask to received data */
					mask--;
					maskval = *mask;
					if (maskval >= '0' && maskval <= '9')
						maskval = maskval - '0';
					else if (maskval >= 'A' &&
					    maskval <= 'F')
						maskval = maskval + 10 - 'A';
					val &= maskval;
					/* Apply mask to expected TDO as well */
					if (tdo != NULL) {
						tdo--;
						tdoval = *tdo;
						if (tdoval >= '0' &&
						    tdoval <= '9')
							tdoval = tdoval - '0';
						else if (tdoval >= 'A' &&
						    tdoval <= 'F')
							tdoval =
							    tdoval + 10 - 'A';
						tdoval &= maskval;
						if (tdoval < 10)
							*tdo = tdoval + '0';
						else
							*tdo =
							    tdoval - 10 + 'A';
					}
				}
				tdi--;
				if (val < 10)
					*tdi = val + '0';
				else
					*tdi = val - 10 + 'A';
				val = 0;
			}
		}
	}

	return (res);
}


static int
send_dr(int bits, char *tdi, char *tdo, char *mask)
{
	int res;

	if (cur_s != DRPAUSE) {
		fprintf(stderr, "Must be in DRPAUSE on entry to send_dr()!\n");
		return (EXIT_FAILURE);
	}
	res = send_generic(bits, tdi, tdo, mask);
	cur_s = DRPAUSE;
	return (res);
}


static int
send_ir(int bits, char *tdi, char *tdo, char *mask)
{
	int res;

	if (cur_s != IRPAUSE) {
		fprintf(stderr, "Must be in IRPAUSE on entry to send_ir()!\n");
		return (EXIT_FAILURE);
	}
	res = send_generic(bits, tdi, tdo, mask);
	cur_s = IRPAUSE;
	return (res);
}


static int
commit_usb(void)
{
	int txchunklen, res, i;

	for (i = 0; i < txpos; i += txchunklen) {
		txchunklen = txpos - i;
		if (port_mode == PORT_MODE_SYNC && txchunklen > USB_BUFLEN_SYNC)
			txchunklen = USB_BUFLEN_SYNC;
#ifdef WIN32
		FT_Write(ftHandle, &txbuf[i], txchunklen, (DWORD *) &res);
#else
		res = ftdi_write_data(&fc, &txbuf[i], txchunklen);
#endif
		if (res != txchunklen) {
			fprintf(stderr, "ftdi_write_data() failed\n");
			return (EXIT_FAILURE);
		}

		if (port_mode == PORT_MODE_SYNC) {
#ifdef WIN32
			FT_Read(ftHandle, &txbuf[i], txchunklen,
			    (DWORD *) &res);
#else
			int rep = 0;
			for (res = 0; res < txchunklen && rep < 8;
			    rep++) {
				res += ftdi_read_data(&fc, &txbuf[i],
				    txchunklen - res);
			}
#endif
			if (res != txchunklen) {
#ifdef WIN32
				fprintf(stderr, "FT_Read() failed: "
				    "expected %d, received %d bytes\n",
				    txchunklen, res);
#else
				fprintf(stderr, "ftdi_read_data() failed\n");
#endif
				return (EXIT_FAILURE);
			}
		}
	}
	txpos = 0;

	/* Schedule CBUS LED blinking */
	i = ms_uptime();
	if (i - last_ledblink_ms >= LED_BLINK_RATE) {
		last_ledblink_ms += LED_BLINK_RATE;
		need_led_blink = 1;
	}

	return (0);
}


#ifdef USE_PPI
static int
commit_ppi(void)
{
	int i, val;

	for (i = 0; i < txpos; i++) {
		val = txbuf[i];
		if (port_mode == PORT_MODE_SYNC && !(i & 1))  {
			ioctl(ppi, PPIGSTATUS, &txbuf[i]);
		}
		ioctl(ppi, PPISDATA, &val);
	}

	txpos = 0;
	return (0);
}
#endif


static int
commit(int force)
{

	if (txpos == 0 || (!force && port_mode != PORT_MODE_SYNC &&
	    txpos < sizeof(txbuf) / 2))
		return (0);

#ifdef USE_PPI
	if (cable_hw == CABLE_HW_PPI)
		return (commit_ppi());
#endif
	if (cable_hw == CABLE_HW_USB)
		return (commit_usb());
	else
		return (EINVAL);
}


static int
str2tapstate(char *str)
{
	int i;

	for (i = 0; tap_statetable[i].state_str != NULL; i++) {
		if (strcmp(str, tap_statetable[i].state_str) == 0)
			break;
	}
	return (tap_statetable[i].state_id);
}


static void
set_state(int tgt_s) {
	int i, res = 0;

	switch (tgt_s) {
	case RESET:
		for (i = 0; i < 5; i++)
			set_tms_tdi(1, 0);
		break;

	case IDLE:
		switch (cur_s) {
		case RESET:
		case DRUPDATE:
		case IRUPDATE:
		case IDLE:
			set_tms_tdi(0, 0);
			break;

		case UNDEFINED:
			set_state(RESET);
			set_state(IDLE);
			break;

		case DRPAUSE:
			set_state(DREXIT2);
			set_state(DRUPDATE);
			set_state(IDLE);
			break;

		case IRPAUSE:
			set_state(IREXIT2);
			set_state(IRUPDATE);
			set_state(IDLE);
			break;

		default:
			res = -1;
		}
		break;

	case DRSELECT:
		switch (cur_s) {
		case IDLE:
		case DRUPDATE:
		case IRUPDATE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case DRCAPTURE:
		switch (cur_s) {
		case DRSELECT:
			set_tms_tdi(0, 0);
			break;

		case IDLE:
			set_state(DRSELECT);
			set_state(DRCAPTURE);
			break;

		case IRPAUSE:
			set_state(IDLE);
			set_state(DRSELECT);
			set_state(DRCAPTURE);
			break;

		default:
			res = -1;
		}
		break;

	case DREXIT1:
		switch (cur_s) {
		case DRCAPTURE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case DRPAUSE:
		switch (cur_s) {
		case DREXIT1:
			set_tms_tdi(0, 0);
			break;

		case IDLE:
			set_state(DRSELECT);
			set_state(DRCAPTURE);
			set_state(DREXIT1);
			set_state(DRPAUSE);
			break;

		case IRPAUSE:
			set_state(IREXIT2);
			set_state(IRUPDATE);
			set_state(DRSELECT);
			set_state(DRCAPTURE);
			set_state(DREXIT1);
			set_state(DRPAUSE);
			break;

		case DRPAUSE:
			set_state(DREXIT2);
			set_state(DRUPDATE);
			set_state(DRSELECT);
			set_state(DRCAPTURE);
			set_state(DREXIT1);
			set_state(DRPAUSE);
			break;

		default:
			res = -1;
		}
		break;

	case DREXIT2:
		switch (cur_s) {
		case DRPAUSE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case DRUPDATE:
		switch (cur_s) {
		case DREXIT2:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case IRSELECT:
		switch (cur_s) {
		case DRSELECT:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case IRCAPTURE:
		switch (cur_s) {
		case IRSELECT:
			set_tms_tdi(0, 0);
			break;

		case IDLE:
			set_state(DRSELECT);
			set_state(IRSELECT);
			set_state(IRCAPTURE);
			break;

		case DRPAUSE:
			set_state(DREXIT2);
			set_state(DRUPDATE);
			set_state(DRSELECT);
			set_state(IRSELECT);
			set_state(IRCAPTURE);
			break;

		default:
			res = -1;
		}
		break;

	case IREXIT1:
		switch (cur_s) {
		case IRCAPTURE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case IRPAUSE:
		switch (cur_s) {
		case IREXIT1:
			set_tms_tdi(0, 0);
			break;

		case IDLE:
			set_state(DRSELECT);
			set_state(IRSELECT);
			set_state(IRCAPTURE);
			set_state(IREXIT1);
			set_state(IRPAUSE);
			break;

		case DRPAUSE:
			set_state(DREXIT2);
			set_state(DRUPDATE);
			set_state(DRSELECT);
			set_state(IRSELECT);
			set_state(IRCAPTURE);
			set_state(IREXIT1);
			set_state(IRPAUSE);
			break;

		case IRPAUSE:
			set_state(IREXIT2);
			set_state(IRUPDATE);
			set_state(DRSELECT);
			set_state(IRSELECT);
			set_state(IRCAPTURE);
			set_state(IREXIT1);
			set_state(IRPAUSE);
			break;

		default:
			res = -1;
		}
		break;

	case IREXIT2:
		switch (cur_s) {
		case IRPAUSE:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	case IRUPDATE:
		switch (cur_s) {
		case IREXIT2:
			set_tms_tdi(1, 0);
			break;

		default:
			res = -1;
		}
		break;

	default:
		res = -1;
	}

	if (res) {
		fprintf(stderr, "Don't know how to proceed: %s -> %s\n",
		    STATE2STR(cur_s), STATE2STR(tgt_s));
		if (cable_hw == CABLE_HW_USB)
			shutdown_usb();
		exit (1);
	}

	cur_s = tgt_s;
}



static int
exec_svf_tokenized(int tokc, char *tokv[])
{
	static int last_sdr = PORT_MODE_UNKNOWN;
	int cmd, i, res = 0;
	int repeat = 1, delay_ms = 0;

	for (i = 0; svf_cmdtable[i].cmd_str != NULL; i++) {
		if (strcmp(tokv[0], svf_cmdtable[i].cmd_str) == 0)
			break;
	}

	cmd = svf_cmdtable[i].cmd_id;
	switch (cmd) {
	case SVF_SDR:
	case SVF_SIR:
		if (tokc == 4) {
			if (cmd == SVF_SDR && last_sdr == PORT_MODE_ASYNC)
				set_port_mode(PORT_MODE_ASYNC);
			tokv[5] = NULL;
			tokv[7] = NULL;
			if (cmd == SVF_SDR)
				last_sdr = PORT_MODE_ASYNC;
		} else if (tokc == 6 || tokc == 8) {
			set_port_mode(PORT_MODE_SYNC);
			if (tokc == 5)
				tokv[7] = NULL;
			if (cmd == SVF_SDR)
				last_sdr = PORT_MODE_SYNC;
		} else {
			res = EXIT_FAILURE;
			break;
		}
		if (cmd == SVF_SDR) {
			set_state(DRPAUSE);
			res = send_dr(atoi(tokv[1]), tokv[3], tokv[5], tokv[7]);
		} else {
			set_state(IRPAUSE);
			res = send_ir(atoi(tokv[1]), tokv[3], tokv[5], tokv[7]);
		}
		if (res)
			break;
		if ((tokc == 6 || tokc == 8) && strcmp(tokv[3], tokv[5]) != 0) {
			fprintf(stderr, "Received and expected data "
			    "do not match!\n");
			if (tokc == 6)
				fprintf(stderr, "TDO: %s Expected: %s\n",
				    tokv[3], tokv[5]);
			if (tokc == 8)
				fprintf(stderr, "TDO: %s Expected: %s "
				    "mask: %s\n", tokv[3], tokv[5], tokv[7]);
			res = EXIT_FAILURE;
		}
		break;

	case SVF_STATE:
		set_state(str2tapstate(tokv[1]));
		res = commit(0);
		break;

	case SVF_RUNTEST:
		for (i = 2; i < tokc; i += 2) {
			if (strcmp(tokv[i + 1], "TCK") == 0) {
				repeat = atoi(tokv[i]);
				if (repeat < 1 || repeat > 1000) {
					fprintf(stderr,
					    "Unexpected token: %s\n",
					    tokv[i]);
					res = EXIT_FAILURE;
					break;
				}
			} else if (strcmp(tokv[i + 1], "SEC") == 0) {
				float f;
				sscanf(tokv[i], "%f", &f);
				delay_ms = (f + 0.0005) * 1000;
				if (delay_ms < 1 || delay_ms > 120000) {
					fprintf(stderr,
					    "Unexpected token: %s\n",
					    tokv[i]);
					res = EXIT_FAILURE;
					break;
				}
				/* Silently reduce insanely long waits */
				if (delay_ms > 3000)
					delay_ms = 3000;
			} else {
				fprintf(stderr, "Unexpected token: %s\n",
				    tokv[i + 1]);
				res = EXIT_FAILURE;
				break;
			}
		}
		set_state(str2tapstate(tokv[1]));
		i = delay_ms * (USB_BAUDS / 2000);
#ifdef USE_PPI
		/* libftdi is relatively slow in sync mode on FreeBSD */
		if (port_mode == PORT_MODE_SYNC && i > USB_BUFLEN_SYNC / 2)
			i /= 2;
#endif
		if (i > repeat)
			repeat = i;
		for (i = 1; i < repeat; i++) {
			txbuf[txpos] = txbuf[txpos-2];
			txpos++;
			txbuf[txpos] = txbuf[txpos-2];
			txpos++;
	    		if (txpos >= sizeof(txbuf) / 2) {
				commit(0);
				if (need_led_blink)
					set_port_mode(port_mode);
			}
		}
		break;

	case SVF_HDR:
	case SVF_HIR:
	case SVF_TDR:
	case SVF_TIR:
		if (tokc != 2 || strcmp(tokv[1], "0") != 0)
			res = EINVAL;
		break;

	case SVF_ENDDR:
		if (tokc != 2 || strcmp(tokv[1], "DRPAUSE") != 0)
			res = EINVAL;
		break;

	case SVF_ENDIR:
		if (tokc != 2 || strcmp(tokv[1], "IRPAUSE") != 0)
			res = EINVAL;
		break;

	case SVF_FREQUENCY:
		/* Silently ignored. */
		break;

	default:
		res = EOPNOTSUPP;
	}

	return (res);
}


enum jed_states {
	JED_INIT, JED_PACK_KNOWN, JED_SIZE_KNOWN, JED_PROG_INITIATED,
	JED_FUSES, JED_FUSES_DONE, JED_SED_CRC, JED_HAVE_SED_CRC, JED_USER_SIG
};

enum jed_target {
	JED_TGT_SRAM, JED_TGT_FLASH, JED_TGT_UNKNOWN
};

static struct jed_devices {
	char	*name;
	int	id;
	int	pincount;
	int	fuses;
	int	row_width;
	int	addr_len;
} jed_devices[] = {
	{
		.name =		"LFXP2-5E-5TQFP144",
		.id =		0x01299043,
		.pincount =	144,
		.fuses =	1236476,
		.row_width =	638,
		.addr_len =	1938,
	},
	{
		.name =		"LFXP2-5E-6TQFP144",
		.id =		0x01299043,
		.pincount =	144,
		.fuses =	1236476,
		.row_width =	638,
		.addr_len =	1938,
	},
	{
		.name =		"LFXP2-5E-7TQFP144",
		.id =		0x01299043,
		.pincount =	144,
		.fuses =	1236476,
		.row_width =	638,
		.addr_len =	1938,
	},
	{
		.name =		"LFXP2-8E-5TQFP144",
		.id =		0x0129A043,
		.pincount =	144,
		.fuses =	1954736,
		.row_width =	772,
		.addr_len =	2532,
	},
	{
		.name =		"LFXP2-8E-6TQFP144",
		.id =		0x0129A043,
		.pincount =	144,
		.fuses =	1954736,
		.row_width =	772,
		.addr_len =	2532,
	},
	{
		.name =		"LFXP2-8E-7TQFP144",
		.id =		0x0129A043,
		.pincount =	144,
		.fuses =	1954736,
		.row_width =	772,
		.addr_len =	2532,
	},
	{NULL, 0, 0}
};

/*
 * Parse a JEDEC file and convert it into SVF program, stored in a
 * contiguos chunk of memory.  If parsing is sucessfull proceed with
 * calling exec_svf_mem().
 */
static int
exec_jedec_file(char *path, int target, int debug)
{
	char *inbuf, *outbuf, *incp, *outcp;
	char tmpbuf[2048];
	FILE *fd;
	long flen;
	int jed_state = JED_INIT;
	int jed_dev = -1;
	int i, j, val, row, res;

	fd = fopen(path, "r");
	if (fd == NULL) {
		fprintf(stderr, "open(%s) failed\n", path);
		return (EXIT_FAILURE);
	}

	fseek(fd, 0, SEEK_END);
	flen = ftell(fd);
	fseek(fd, 0, SEEK_SET);

	inbuf = malloc(flen);
	outbuf = malloc(flen * 2); /* XXX rough estimate */
	if (inbuf == NULL || outbuf == NULL) {
		fprintf(stderr, "malloc(%ld) failed\n", flen);
		return (EXIT_FAILURE);
	}

	incp = inbuf;
	outcp = outbuf;
	while (!feof(fd) && fgets(incp, flen, fd) != NULL) {
		/* Trim CR / LF chars from the tail of the line */
		incp += strlen(incp) - 1;
		while (incp >= inbuf && (*incp == 10 || *incp == 13))
			incp--;
		incp[1] = 0;

		/* Is this the first line of an "L" command? */
		if (*inbuf == 'L') {
			if (jed_state < JED_PROG_INITIATED) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}
			if (jed_state == JED_PROG_INITIATED)
				jed_state = JED_FUSES;
			else
				jed_state = JED_SED_CRC;
			incp = inbuf;
			continue;
		}

		/* Does the command terminate on this line? */
		if (*incp != '*') {
			incp++;
			continue;
		} else
			*incp = 0;

		/* Is this the SED_CRC fuses string? */
		if (jed_state == JED_SED_CRC) {
			val = 0;
			for (i = 32, j = 0; i > 0; i--, val <<= 1) {
				val += (inbuf[i - 1] == '1');
				if ((i & 0x3) == 1) {
					if (val < 10)
						tmpbuf[j++] = '0' +
						    val;
					else
						tmpbuf[j++] = 'A' +
						    val - 10;
					val = 0;
				}
			}
			tmpbuf[j++] = 0;
			if (strlen(tmpbuf) != 8) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}
			jed_state = JED_HAVE_SED_CRC;
		}

		/* Is this the main fuses string? */
		if (jed_state == JED_FUSES) {

			outcp += sprintf(outcp, "\n\n! Program Fuse Map\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (21);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-002 SEC;\n");
			*outcp++ = 0;

			if (target == JED_TGT_SRAM) {
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (67);\n");
				*outcp++ = 0;
			}

			for (incp = inbuf, row = 1;
			    row <= jed_devices[jed_dev].addr_len; row++) {
				if (target == JED_TGT_FLASH) {
					outcp += sprintf(outcp,
					    "SIR	8	TDI  (67);\n");
					*outcp++ = 0;
				}

				val = 0;
				for (i = jed_devices[jed_dev].row_width, j = 0;
				    i > 0; i--, val <<= 1) {
					val += (incp[i - 1] == '1');
					if ((i & 0x3) == 1) {
						if (val < 10)
							tmpbuf[j++] = '0' +
							    val;
						else
							tmpbuf[j++] = 'A' +
							    val - 10;
						val = 0;
					}
				}
				tmpbuf[j++] = 0;
				incp += jed_devices[jed_dev].row_width;

				outcp += sprintf(outcp,
				    "! Shift in Data Row = %d\n", row);
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	%d	TDI  (%s);\n",
				    jed_devices[jed_dev].row_width, tmpbuf);
				*outcp++ = 0;
				if (target == JED_TGT_FLASH) {
					outcp += sprintf(outcp,
					    "RUNTEST	IDLE"
					    "	3 TCK	1.00E-003 SEC;\n");
				} else {
					outcp += sprintf(outcp,
					    "RUNTEST	IDLE	3 TCK;\n");
				}
				*outcp++ = 0;

				if (target == JED_TGT_FLASH) {
					outcp += sprintf(outcp,
					    "SIR	8	TDI  (52);\n");
					*outcp++ = 0;

					outcp += sprintf(outcp,
					    "SDR	1	TDI  (0)\n");
					*outcp++ = 0;
					outcp += sprintf(outcp,
					    "		TDO  (1);\n");
					*outcp++ = 0;
				}
			}

			/* Check that we have consumed all fuse bits */
			if (strlen(incp) != 0) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}

			jed_state++;
		}
		
		/* Is this a comment line? */
		if (*inbuf == 'N') {
			if (jed_state == JED_INIT) {
				outcp += sprintf(outcp, "! %s\n", inbuf);
				*outcp++ = 0;
			}
			if (strncmp(inbuf, "NOTE DEVICE NAME:", 17) == 0) {
				incp = &inbuf[18];
				for (jed_dev = 0;
				    jed_devices[jed_dev].name != NULL;
				    jed_dev++) {
					if (strcmp(jed_devices[jed_dev].name,
					    incp) == 0)
						break; 
				}
				if (jed_devices[jed_dev].name == NULL) {
					fprintf(stderr, "Bitstream for "
					    "unsupported target: %s\n", incp);
					return (EXIT_FAILURE);
				}
			}
			incp = inbuf;
			continue;
		}

		/* Packaging line? */
		if (*inbuf == 'Q') {
			i = atoi(&inbuf[2]);
			if (inbuf[1] == 'P') {
				if (jed_dev < 0 || jed_state != JED_INIT
				    || jed_devices[jed_dev].pincount != i) {
					fprintf(stderr,
					    "Invalid bitstream file\n");
					return (EXIT_FAILURE);
				}
				jed_state = JED_PACK_KNOWN;
			} else if (inbuf[1] == 'F') {
				if (jed_dev < 0 || jed_state != JED_PACK_KNOWN
				    || jed_devices[jed_dev].fuses != i) {
					fprintf(stderr,
					    "Invalid bitstream file\n");
					return (EXIT_FAILURE);
				}
				jed_state = JED_SIZE_KNOWN;
			} else {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}
		}

		/* "F" line? */
		if (*inbuf == 'F') {
			if (jed_state != JED_SIZE_KNOWN) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}
			jed_state = JED_PROG_INITIATED;

			outcp += sprintf(outcp, "\n\n! Check the IDCODE\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "STATE	RESET;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "STATE	IDLE;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (16);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "SDR	32	TDI  (FFFFFFFF)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		TDO  (%08X)\n",
			    jed_devices[jed_dev].id);
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "		MASK (FFFFFFFF);\n");
			*outcp++ = 0;

			if (target == JED_TGT_SRAM) {
				outcp += sprintf(outcp,
				    "\n\n! Program Bscan register\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (1C);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "STATE	DRPAUSE;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "STATE	IDLE;\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "\n\n! Enable SRAM programming mode\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (55);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "\n\n! Erase the device\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (03);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
			} else {
				outcp += sprintf(outcp,
				    "\n\n! Enable XPROGRAM mode\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (35);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "\n\n! Check the Key Protection fuses\n\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (10);\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (40);\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (04);\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "\n\n! Erase the device\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (03);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.20E+002 SEC;\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (52);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	1	TDI  (0)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (1);\n");
				*outcp++ = 0;

				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (01);\n");
				*outcp++ = 0;
			}
		}

		/* "U" line? */
		if (*inbuf == 'U') {
			if (inbuf[1] != 'H' || jed_state != JED_HAVE_SED_CRC) {
				fprintf(stderr, "Invalid bitstream file\n");
				return (EXIT_FAILURE);
			}

			outcp += sprintf(outcp, "\n\n! Program USERCODE\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (1A);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "SDR	32	TDI  (%s);\n", &inbuf[2]);
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-002 SEC;\n");
			*outcp++ = 0;

			if (target == JED_TGT_FLASH) {
				outcp += sprintf(outcp,
				    "\n\n! Read the status bit;\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2);\n");
				*outcp++ = 0;
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	1.00E-003 SEC;\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SDR	8	TDI  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (00)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (01);\n");
				*outcp++ = 0;
			}

			outcp += sprintf(outcp,
			    "\n\n! Program and Verify 32 bits SED_CRC\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (45);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "SDR	32	TDI  (%s);\n", tmpbuf);
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-002 SEC;\n");
			*outcp++ = 0;

			outcp += sprintf(outcp, "SIR	8	TDI  (44);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-003 SEC;\n");
			*outcp++ = 0;

			outcp += sprintf(outcp,
			    "SDR	32	TDI  (00000000)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "		TDO  (%s);\n", tmpbuf);
			*outcp++ = 0;

			outcp += sprintf(outcp, "SIR	8	TDI  (B2);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-003 SEC;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SDR	8	TDI  (00)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		TDO  (00)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		MASK (01);\n");
			*outcp++ = 0;

			outcp += sprintf(outcp,
			    "\n\n! Program DONE bit\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (2F);\n");
			*outcp++ = 0;
			if (target == JED_TGT_FLASH) {
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK	2.00E-001 SEC;\n");
			} else {
				outcp += sprintf(outcp, "RUNTEST	IDLE"
				    "	3 TCK;\n");
			}
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (B2);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-003 SEC;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SDR	8	TDI  (00)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		TDO  (02)\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "		MASK (03);\n");
			*outcp++ = 0;

			if (target == JED_TGT_FLASH) {
				outcp += sprintf(outcp,
				    "\n\n! Verify DONE bit\n\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "SIR	8	TDI  (B2)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		TDO  (FF)\n");
				*outcp++ = 0;
				outcp += sprintf(outcp,
				    "		MASK (04);\n");
				*outcp++ = 0;
			}

			outcp += sprintf(outcp,
			    "\n\n! Exit the programming mode\n\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (1E);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	2.00E-003 SEC;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp, "SIR	8	TDI  (FF);\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "RUNTEST	IDLE	3 TCK	1.00E-003 SEC;\n");
			*outcp++ = 0;
			outcp += sprintf(outcp,
			    "STATE	RESET;\n");
			*outcp++ = 0;
		}

		incp = inbuf;
	}
	fclose(fd);

	/* Count number of lines in outbuf, store in j */
	for (i = 0, j = 0; outcp > outbuf; outcp--)
		if (*outcp == 0)
			j++;

	res = exec_svf_mem(outbuf, j, debug);

	free(outbuf);
	free(inbuf);
	return (res);
}


/*
 * Load a SVF file in a contiguos chunk of memory, count number of lines,
 * and then call exec_svf_mem().
 */
static int
exec_svf_file(char *path, int debug)
{
	char *linebuf, *fbuf;
	FILE *fd;
	long flen;
	int lines_tot = 0;
	int res;

	fd = fopen(path, "r");
	if (fd == NULL) {
		fprintf(stderr, "open(%s) failed\n", path);
		return (EXIT_FAILURE);
	}

	fseek(fd, 0, SEEK_END);
	flen = ftell(fd);
	fseek(fd, 0, SEEK_SET);

	fbuf = malloc(flen);
	if (fbuf == NULL) {
		fprintf(stderr, "malloc(%ld) failed\n", flen);
		return (EXIT_FAILURE);
	}

	for (linebuf = fbuf; !feof(fd); linebuf += strlen(linebuf) + 1) {
		if (fgets(linebuf, flen, fd) == NULL)
			break;
		lines_tot++;
	}
	fclose(fd);

	res = exec_svf_mem(fbuf, lines_tot, debug);
	free(fbuf);
	return (res);
}


/*
 * Parse SVF command lines stored in a contiguos chunk of memory and
 * execute appropriate JTAG actions, line by line, all in a single pass.
 */
static int
exec_svf_mem(char *fbuf, int lines_tot, int debug)
{
	char cmdbuf[4096];
	int lno, tokc, cmd_complete, parentheses_open;
	int res = 0;
	int llen = 0;
	char *cp, *c1;
	char *sep = " \t\n\r";
	char *linebuf, *item, *brkt;
	char *tokv[256];

	cp = cmdbuf;
	cmd_complete = 0;
	parentheses_open = 0;
	linebuf = fbuf;

	for (lno = 1; lno < lines_tot; lno++, linebuf += llen) {
		if (debug)
			printf("%s", linebuf);

		llen = strlen(linebuf) + 1;
		progress_perc = lno * 1005 / (lines_tot * 10);

		/* Pre-parse input, join multiple lines to a single command */
		for (item = strtok_r(linebuf, sep, &brkt); item;
		    item = strtok_r(NULL, sep, &brkt)) {
			/* Skip comments */
			if (*item == '!')
				break;

			/* If command is complete we shouldn't end up here! */
			if (cmd_complete) {
				fprintf(stderr, "Line %d: multiple commands"
				    "on single line\n", lno);
				return (EXIT_FAILURE);
			}

			/* End of command? */
			c1 = item + strlen(item) - 1;
			if (*c1 == ';') {
				*c1-- = 0;
				cmd_complete = 1;
			}

			/* Check for parentheses */
			if (*item == '(') {
				item++;
				if (parentheses_open) {
					fprintf(stderr,
					    "Line %d: too many '('s\n", lno);
					return (EXIT_FAILURE);
				}
				parentheses_open = 1;
			}
			if (*c1 == ')') {
				*c1 = 0;
				if (!parentheses_open) {
					fprintf(stderr,
					    "Line %d: too many ')'s\n", lno);
					return (EXIT_FAILURE);
				}
				parentheses_open = 0;
			}

			/* Copy to command buffer */
			strcpy(cp, item);
			cp += strlen(item);
			if (!parentheses_open && !cmd_complete)
				*cp++ = ' ';
		}

		/* Proceed to next line if command is not complete yet */
		if (!cmd_complete)
			continue;

		/* Unmatched parentheses are not permitted */
		if (parentheses_open) {
			fprintf(stderr, "Line %d: missing ')'\n", lno);
			return (EXIT_FAILURE);
		}

		/* Normalize to all upper case letters, separate tokens */
		tokc = 0;
		tokv[0] = cmdbuf;
		for (cp = cmdbuf; *cp != 0; cp++) {
			if (*cp == ' ') {
				*cp++ = 0;
				tokc++;
				tokv[tokc] = cp;
			}
			*cp = toupper(*cp);
		}
		tokc++;

		/* Execute command */
		res = exec_svf_tokenized(tokc, tokv);
		if (res) {
			fprintf(stderr, "Line %d: %s\n", lno, strerror(res));
			return (EXIT_FAILURE);
		}

		cp = cmdbuf;
		cmd_complete = 0;
	}

	/* Flush any buffered data */
	commit(1);

	return (res);
}


static void
usage(void)
{

	printf(
#ifdef USE_PPI
	    "Usage: ujprog [-td%s] [-c usb|ppi] [-j sram|flash] file\n",
#else
	    "Usage: ujprog [-td%s] [-j sram|flash] file\n",
#endif
#ifdef WIN32
	    "s"
#else
	    ""
#endif
	    );
}


static int
gets1(char *cp, int size)
{
	char *lp, *end;
	char c;
	int error = 0;

	lp = cp;
	end = cp + size - 1;
	for (;;) {
#ifdef WIN32
		c = getch() & 0177;
#else
		while (read(0, &c, 1) < 0)
			ms_sleep(10);
#endif
		switch (c) {
		case 3:	/* CTRL + C */
			error = -1;
		case '\n':
		case '\r':
			printf("\n");
			*lp = '\0';
			return (error);
		case '\b':
		case '\177':
			if (lp > cp) {
				printf("%c \b", c);
				fflush(stdout);
				lp--;
			}
			continue;
		case '\0':
			continue;
		default:
			if (lp < end) {
				printf("%c", c);
				fflush(stdout);
				*lp++ = c;
			}
		}
	}
}


static int
prog(char *fname, int jed_target, int debug)
{
	int res, c, tstart, tend;

	tstart = ms_uptime();
	last_ledblink_ms = tstart;

	/* Move TAP into RESET state. */
	set_port_mode(PORT_MODE_ASYNC);
	set_state(IDLE);
	set_state(RESET);

	commit(1);

	c = strlen(fname) - 4;
	if (c < 0) {
		usage();
		exit (EXIT_FAILURE);
	}
	if (strcasecmp(&fname[c], ".jed") == 0)
		res = exec_jedec_file(fname, jed_target, debug);
	else
		res = exec_svf_file(fname, debug);

	/* Leave TAP in RESET state. */
	set_port_mode(PORT_MODE_ASYNC);
	set_state(IDLE);
	set_state(RESET);
	commit(1);

	tend = ms_uptime();
	if (res == 0) {
		printf("\rProgramming: 100%%  ");
		printf("\nCompleted in %.2f seconds.\n",
		    (tend - tstart) / 1000.0);
	} else
		printf("\nFailed.\n");

	return (res);
}


static int
term_emul(void)
{
#ifdef WIN32
	DWORD saved_cons_mode;
	DWORD rx_cnt, tx_cnt, ev_stat, sent;
	HANDLE cons_in = GetStdHandle(STD_INPUT_HANDLE);
	HANDLE cons_out = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_CURSOR_INFO cursor_info;
#else
	int rx_cnt, tx_cnt, sent;
#endif
	int key_phase = 1; /* 0 .. normal; 1 .. CR; 2 .. CR + ~ */
	int c, res;
	int infile = -1;
	char argbuf[256];
	
	printf("Entering terminal emulation mode using %d bauds\n", bauds);

	set_port_mode(PORT_MODE_UART);
#ifdef WIN32
	FT_SetLatencyTimer(ftHandle, 20);
	FT_SetBaudRate(ftHandle, bauds);
	FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1,
	    FT_PARITY_NONE);
	FT_SetFlowControl(ftHandle, FT_FLOW_NONE, 0, 0);
	do {} while (FT_StopInTask(ftHandle) != FT_OK);
	FT_Purge(ftHandle, FT_PURGE_RX);
	do {} while (FT_RestartInTask(ftHandle) != FT_OK);

	/* Disable CTRL-C, XON/XOFF etc. processing on console input. */
	GetConsoleMode(cons_in, &saved_cons_mode);
	SetConsoleMode(cons_in, 0);
	cursor_info.bVisible = 1;
	cursor_info.dwSize = 100;
	SetConsoleCursorInfo(cons_out, &cursor_info);
#else
	ftdi_set_latency_timer(&fc, 20);
	ftdi_set_baudrate(&fc, bauds);
	ftdi_set_line_property(&fc, BITS_8, STOP_BIT_1, NONE);
	ftdi_setflowctrl(&fc, SIO_DISABLE_FLOW_CTRL);
	ftdi_usb_purge_buffers(&fc);

	/* Disable CTRL-C, XON/XOFF etc. processing on console input. */
	fcntl(0, F_SETFL, O_NONBLOCK);
	system("stty -echo -isig -icanon -iexten -ixon -ixoff -icrnl");
#endif

	do {
		tx_cnt = 0;
		if (infile > 0) {
			res = read(infile, txbuf, 2048);
			if (res <= 0) {
				close(infile);
				infile = -1;
				tx_cnt = 0;
			} else
				tx_cnt = res;
		} else
#ifdef WIN32
		while (kbhit()) {
			c = getch();
			txbuf[tx_cnt] = c;
#else
		while (read(0, &txbuf[tx_cnt], 1) > 0) {
			c = txbuf[tx_cnt];
#endif
			if (key_phase == 2) {
				switch (c) {
				case '?':
					printf("~?\n"
					    " ~>	send file\n"
					    " ~b	change baudrate\n"
					    " ~r	reprogram the FPGA\n"
					    " ~.	exit from ujprog\n"
					    " ~?	get this summary\n"
					);
					continue;
				case 'r':
					res = 0;
					goto done;
				case '.':
					res = 1;
					goto done;
				case 'b':
					printf("~>New baudrate? ");
					fflush(stdout);
					gets1(argbuf, sizeof(argbuf));
					c = atoi(argbuf);
					if (c > 0) {
#ifdef WIN32
						res = FT_SetBaudRate(ftHandle,
						    c);
						if (res == FT_OK) {
#else
						res = ftdi_set_baudrate(&fc,
						    c);
						if (res == 0) {
#endif
							bauds = c;
							printf("new"
							    " baudrate: %d\n",
							    bauds);
						} else
							printf("%d: invalid"
							    " baudrate\n", c);
					}
					key_phase = 0;
					continue;
				case '>':
					printf("~>Local file name? ");
					fflush(stdout);
					gets1(argbuf, sizeof(argbuf));
					infile = open(argbuf,
#ifdef WIN32
					    O_RDONLY | O_BINARY
#else
					    O_RDONLY
#endif
					    );
					if (infile < 0)
						printf("%s: cannot open\n",
						    argbuf);
					key_phase = 0;
					continue;
				default:
					if (c != '~') {
						txbuf[tx_cnt] = '~';
						tx_cnt++;
						txbuf[tx_cnt] = c;
					}
					key_phase = 0;
					break;
				}
			}
			if (key_phase == 1 && c == '~') {
				key_phase = 2;
				continue;
			}
			if (c == 13)
				key_phase = 1;
			else
				key_phase = 0;
			tx_cnt++;
		}
		if (tx_cnt) {
#ifdef WIN32
			FT_Write(ftHandle, txbuf, tx_cnt, &sent);
#else
			sent = ftdi_write_data(&fc, txbuf, tx_cnt);
#endif
			if (sent != tx_cnt)
				printf("XXX USB req %d sent %d\n",
				    tx_cnt, sent);
		}

#ifdef WIN32
		FT_GetStatus(ftHandle, &rx_cnt, &ev_stat, &ev_stat);
		if (rx_cnt) {
			FT_Read(ftHandle, txbuf, rx_cnt, &rx_cnt);
#else
		rx_cnt = ftdi_read_data(&fc, txbuf, 128);
		if (rx_cnt) {
#endif
			fwrite(txbuf, rx_cnt, 1, stdout);
			fflush(stdout);
		}
		if (rx_cnt == 0)
			ms_sleep(10);
	} while (1);

done:
	printf("\n");

	/* Restore special key processing on console input. */
#ifdef WIN32
	SetConsoleMode(cons_in, saved_cons_mode);
	cursor_info.bVisible = 1;
	cursor_info.dwSize = 20;
	SetConsoleCursorInfo(cons_out, &cursor_info);
	FT_SetLatencyTimer(ftHandle, 1);
	FT_SetBaudRate(ftHandle, USB_BAUDS);
#else
	system("stty echo isig icanon iexten ixon ixoff icrnl");
	ftdi_set_latency_timer(&fc, 1);
	ftdi_set_baudrate(&fc, USB_BAUDS);
#endif

	return (res);
}


int
main(int argc, char *argv[])
{
	int res = EXIT_FAILURE;
	int jed_target = JED_TGT_SRAM;
	int terminal = 0;
	int debug = 0;
	int c;

	printf("ULX2S JTAG programmer v 1.02 2013/08/03 (zec)\n");

#ifdef WIN32
#define OPTS	"tsdc:j:b:"
#else
#define OPTS	"tdj:b:"
#endif
	while ((c = getopt(argc, argv, OPTS)) != -1) {
		switch (c) {
		case 'd':
			debug = 1;
			break;
		case 't':
			terminal = 1;
			break;
		case 'b':
			bauds = atoi(optarg);
			break;
#ifdef USE_PPI
		case 'c':
			if (strcmp(optarg, "usb") == 0)
				cable_hw = CABLE_HW_USB;
			else if (strcmp(optarg, "ppi") == 0)
				cable_hw = CABLE_HW_PPI;
			else {
				usage();
				exit (EXIT_FAILURE);
			}
			break;
#endif
		case 'j':
			if (strcmp(optarg, "sram") == 0)
				jed_target = JED_TGT_SRAM;
			else if (strcmp(optarg, "flash") == 0)
				jed_target = JED_TGT_FLASH;
			else {
				usage();
				exit (EXIT_FAILURE);
			}
			break;
#ifdef WIN32
		case 's':
			quick_mode = 0;
			break;
#endif
		case '?':
		default:
			usage();
			exit (EXIT_FAILURE);
		}
	}
	argc -= optind;
	argv += optind;

	if (argc == 0 && terminal == 0) {
		usage();
		exit (EXIT_FAILURE);
	};

	if (argc > 0) {
	}

	switch (cable_hw) {
	case CABLE_HW_UNKNOWN:
	case CABLE_HW_USB:
		res = setup_usb();
		if (res == 0)
			cable_hw = CABLE_HW_USB;
		if (cable_hw == CABLE_HW_USB)
			break;
#ifdef USE_PPI
	case CABLE_HW_PPI:
		res = setup_ppi();
#endif
	default:
		/* can't happen, shut up gcc warnings */
		break;
	}

	if (res) {
		fprintf(stderr, "Cannot find JTAG cable.\n");
		exit (EXIT_FAILURE);
	}

#ifndef WIN32
	if (cable_hw == CABLE_HW_USB)
		printf("Using USB JTAG cable.\n");
	else
#ifdef USE_PPI
		printf("Using parallel port JTAG cable.\n");
#else
		printf("Parallel port JTAG cable not supported!\n");
#endif
#endif /* !WIN32 */

	do {
		if (argc)
			prog(argv[0], jed_target, debug);
	} while (terminal && term_emul() == 0);

	if (cable_hw == CABLE_HW_USB)
		shutdown_usb();
#ifdef USE_PPI
	else
		shutdown_ppi();
#endif

	return (res);
}
