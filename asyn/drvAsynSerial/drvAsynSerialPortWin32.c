/**********************************************************************
* Asyn device support using local serial interface on WIN32
*
* Mark Rivers
* July 26, 2011
**********************************************************************/       
/***********************************************************************
* Copyright (c) 2002 The University of Chicago, as Operator of Argonne
* National Laboratory, and the Regents of the University of
* California, as Operator of Los Alamos National Laboratory, and
* Berliner Elektronenspeicherring-Gesellschaft m.b.H. (BESSY).
* asynDriver is distributed subject to a Software License Agreement
* found in file LICENSE that is included with this distribution.
***********************************************************************/

/*
 * $Id: drvAsynSerialPort.c,v 1.49 2009-08-13 20:35:31 norume Exp $
 */

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <windows.h>

#include <osiUnistd.h>
#include <cantProceed.h>
#include <errlog.h>
#include <iocsh.h>
#include <epicsAssert.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <osiUnistd.h>

#include <epicsExport.h>
#include "asynDriver.h"
#include "asynOctet.h"
#include "asynOption.h"
#include "asynInterposeEos.h"
#include "drvAsynSerialPort.h"

/* 
 * convert windows error code from GetLastError() to malloced string 
 */
static char* getLastErrorMessage(DWORD error)
{
    char* mess;
    LPVOID lpMsgBuf = NULL;
    if (FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        error,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &lpMsgBuf,
        0, NULL ) == 0)
    {
        return strdup("<unknown>");
    }            
    mess = strdup(lpMsgBuf);
    LocalFree(lpMsgBuf);
    return mess;
}

static void getTimestamp(char* buffer, size_t nbuff)
{
	time_t now;
	time(&now);
	strftime(buffer, nbuff, "%Y-%m-%d %H:%M:%S", localtime(&now));
}

static void printCOMError(const char* serialDeviceName, DWORD error, FILE* fp)
{
	char datetime[64];
	getTimestamp(datetime, sizeof(datetime));
	fprintf(fp, "%s: %s COM error code %d was present\n", datetime, serialDeviceName, error);
	if (error & CE_BREAK) {
		fprintf(fp, "    The hardware detected a break condition\n");
	}
	if (error & CE_FRAME) {
		fprintf(fp, "    The hardware detected a framing error\n");
	}
	if (error & CE_OVERRUN) {
		fprintf(fp, "    A character-buffer overrun has occurred\n");
	}
	if (error & CE_RXOVER) {
		fprintf(fp, "    An input buffer overflow has occurred\n");
	}
	if (error & CE_RXPARITY) {
		fprintf(fp, "    The hardware detected a parity error\n");
	}
}


/*
 * This structure holds the hardware-specific information for a single
 * asyn link.  There is one for each serial line.
 */
typedef struct {
    asynUser          *pasynUser;
    char              *serialDeviceName;
    char              *portName;
    unsigned long      nRead;
    unsigned long      nWritten;
    int                baud;
    HANDLE             commHandle;
	HANDLE			   commEventHandle;     /* event for overlapped IO in ReadFile and WriteFile */
	HANDLE			   commEventMaskHandle; /* event for overlapped IO in WaitCommEvent */
	DWORD              commEventMask;       /* mask for WaitCommEvent */
	OVERLAPPED         commOverlapped;      /* for ReadFile and WriteFile */
	OVERLAPPED         commEventOverlapped; /* for WaitCommEvent */
    COMMCONFIG         commConfig;
	HANDLE             bytesAvailableEvent;
	int                printCharEvent;
    double             readTimeout;
    double             writeTimeout;
    epicsTimerId       timer;
    volatile int       timeoutFlag;
    unsigned           break_delay;     /* length of sleep after sending bytes (ms). If both are defined sleep happens before break. */
    unsigned           break_duration;  /* length of serial break to send after a write (ms) */
    int                flush_on_write;  /* whether to flush data after every write */
    asynInterface      common;
    asynInterface      option;
    asynInterface      octet;
} ttyController_t;

typedef struct serialBase {
    epicsTimerQueueId timerQueue;
} serialBase;
static serialBase *pserialBase = 0;
static void serialBaseInit(void)
{
    if(pserialBase) return;
    pserialBase = callocMustSucceed(1,sizeof(serialBase),"serialBaseInit");
    pserialBase->timerQueue = epicsTimerQueueAllocate(
        1,epicsThreadPriorityScanLow);
}

// number of bytes or -1 on error
static int bytesAtPort(ttyController_t *tty, asynUser* pasynUser)
{
	DWORD error;
	BOOL ret;
	COMSTAT cstat;
	ret = ClearCommError(tty->commHandle, &error, &cstat);
	if (ret == 0) {
		error = GetLastError();
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
			"Can't clear \"%s\" error=%d", tty->serialDeviceName, error);
		return -1;
	}
	if (error != 0)
	{
		printCOMError(tty->serialDeviceName, error, stdout);
	}
	return cstat.cbInQue;
}

// number of bytes or -1 on error
static int waitForBytes(ttyController_t *tty, asynUser* pasynUser, double timeout, int nbytes)
{
	int n;
	DWORD dwRes, error;
	epicsTimeStamp ts, ts_start;
	epicsTimeGetCurrent(&ts_start);
	n = bytesAtPort(tty, pasynUser);
	while(n != -1 && n < nbytes && timeout > 0.0)
	{
		dwRes = WaitForSingleObject(tty->bytesAvailableEvent, (int)(timeout * 1000.0));
		error = GetLastError(); /* read now as may need later and bytesAtPort() could change this */
		n = bytesAtPort(tty, pasynUser);
		if (dwRes == WAIT_TIMEOUT || n < 0)
		{
		    return n;
		}
		else if (dwRes == WAIT_OBJECT_0)
		{
			epicsTimeGetCurrent(&ts);
			timeout -= epicsTimeDiffInSeconds(&ts, &ts_start);
		}
		else
		{
			/* error code read earlier */
			epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
				"waitForBytes: WaitForSingleObject error=%d", error);
			return -1;
		}
	}
	return n;
}

/* monitor and print requested comm events */
static void monitorComEvents(void* arg)
{
	char datetime[64];
	ttyController_t *tty = (ttyController_t *)arg;
	DWORD evtMask, error, readTotal;
	asynPrint(tty->pasynUser, ASYN_TRACE_FLOW,
		"%s started monitorComEvents thread.\n", tty->serialDeviceName);
	while(1)
	{
		if (GetCommMask(tty->commHandle, &evtMask) == 0 || evtMask == 0)
		{
			break; /* nothing to monitor, also used to indicate connection close as by default we monitor EV_ERR */
		}
	    memset(&tty->commEventOverlapped, 0, sizeof(OVERLAPPED));
	    tty->commEventOverlapped.hEvent = tty->commEventMaskHandle;
	    if (WaitCommEvent(tty->commHandle, &evtMask, &tty->commEventOverlapped) == 0)
		{
			error = GetLastError();
			if (error != ERROR_IO_PENDING)
			{
				break; /* terminates thread */
			}
			if (GetOverlappedResult(tty->commHandle, &tty->commEventOverlapped, &readTotal, TRUE) == 0)
			{
				break; /* terminates thread */
			}
		}
		getTimestamp(datetime, sizeof(datetime));
		if (evtMask & EV_ERR)
		{
			printf("%s: %s COM event: line status error: frame, overrun or parity error\n", datetime, tty->serialDeviceName);
		}
		if (evtMask & EV_CTS)
		{
			printf("%s: %s COM event: CTS state change\n", datetime, tty->serialDeviceName);
		}
		if (evtMask & EV_DSR)
		{
			printf("%s: %s COM event: DSR state change\n", datetime, tty->serialDeviceName);
		}
		if (evtMask & EV_BREAK)
		{
			printf("%s: %s COM event: break detected\n", datetime, tty->serialDeviceName);
		}
		if (evtMask & EV_RLSD)
		{
			printf("%s: %s COM event: DCD/RLSD state change\n", datetime, tty->serialDeviceName);
		}
		if (evtMask & EV_RING)
		{
			printf("%s: %s COM event: ring indicator detected\n", datetime, tty->serialDeviceName);
		}
		if (evtMask & EV_RXCHAR)
		{
			SetEvent(tty->bytesAvailableEvent);
			if (tty->printCharEvent != 0)
			{
			    printf("%s: %s COM event: character received and placed in input buffer\n", datetime, tty->serialDeviceName);
			}
		}
		if (evtMask & EV_RXFLAG)
		{
			printf("%s: %s COM event: event character received\n", datetime, tty->serialDeviceName);
		}
		if (evtMask & EV_TXEMPTY)
		{
			printf("%s: %s COM event: last character sent from output buffer\n", datetime, tty->serialDeviceName);
		}
	}
	asynPrint(tty->pasynUser, ASYN_TRACE_FLOW,
		"%s terminated monitorComEvents thread.\n", tty->serialDeviceName);
}

/*
 * asynOption methods
 */
static asynStatus
getOption(void *drvPvt, asynUser *pasynUser,
                              const char *key, char *val, int valSize)
{
    ttyController_t *tty = (ttyController_t *)drvPvt;
    DWORD commConfigSize = sizeof(tty->commConfig);
    COMMPROP commprop;
    BOOL ret;
    DWORD error;
    int l;
    
    val[0] = '\0';
    assert(tty);
    if (tty->commHandle == INVALID_HANDLE_VALUE) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s disconnected:", tty->serialDeviceName);
        return asynError;
    }
    ret = GetCommConfig(tty->commHandle, &tty->commConfig, &commConfigSize);
    if (ret == 0) {
        error = GetLastError();
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                            "%s error calling GetCommConfig %d", tty->serialDeviceName, error);
        return asynError;
    }
    ret = GetCommProperties(tty->commHandle, &commprop);
    if (ret == 0) {
        error = GetLastError();
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                            "%s error calling GetCommProperties %d", tty->serialDeviceName, error);
        return asynError;
    }

    if (epicsStrCaseCmp(key, "baud") == 0) {
        l = epicsSnprintf(val, valSize, "%d", tty->commConfig.dcb.BaudRate);
    }
    else if (epicsStrCaseCmp(key, "bits") == 0) {
        l = epicsSnprintf(val, valSize, "%d", tty->commConfig.dcb.ByteSize);
    }
    else if (epicsStrCaseCmp(key, "parity") == 0) {
        switch (tty->commConfig.dcb.Parity) {
            case 0:
                l = epicsSnprintf(val, valSize, "none");
                break;
            case 1:
                l = epicsSnprintf(val, valSize, "odd");
                break;
            case 2:
                l = epicsSnprintf(val, valSize, "even");
                break;
            case 3:
                l = epicsSnprintf(val, valSize, "mark");
                break;
            case 4:
                l = epicsSnprintf(val, valSize, "space");
                break;
        }
    }
    else if (epicsStrCaseCmp(key, "stop") == 0) {
        l = epicsSnprintf(val, valSize, "%.3g", 1.0 + tty->commConfig.dcb.StopBits / 2.0);
    }
    else if (epicsStrCaseCmp(key, "clocal") == 0) {
        l = epicsSnprintf(val, valSize, "%c",  (tty->commConfig.dcb.fOutxDsrFlow == TRUE) ? 'N' : 'Y');
    }
    else if (epicsStrCaseCmp(key, "crtscts") == 0) {
        l = epicsSnprintf(val, valSize, "%c",  (tty->commConfig.dcb.fOutxCtsFlow == TRUE) ? 'Y' : 'N');
    }
    else if (epicsStrCaseCmp(key, "ixon") == 0) {
        l = epicsSnprintf(val, valSize, "%c",  (tty->commConfig.dcb.fOutX == TRUE) ? 'Y' : 'N');
    }
    else if (epicsStrCaseCmp(key, "ixany") == 0) {
        l = epicsSnprintf(val, valSize, "%c",  'N');
    }
    else if (epicsStrCaseCmp(key, "ixoff") == 0) {
        l = epicsSnprintf(val, valSize, "%c",  (tty->commConfig.dcb.fInX == TRUE) ? 'Y' : 'N');
    }
    else if (epicsStrCaseCmp(key, "break_duration") == 0) {
        l = epicsSnprintf(val, valSize, "%u",  tty->break_duration);
    }
    else if (epicsStrCaseCmp(key, "break_delay") == 0) {
        l = epicsSnprintf(val, valSize, "%u",  tty->break_delay);
    }
    else if (epicsStrCaseCmp(key, "rbuff") == 0) {
        l = epicsSnprintf(val, valSize, "%d",  commprop.dwCurrentRxQueue);
    }
    else if (epicsStrCaseCmp(key, "wbuff") == 0) {
        l = epicsSnprintf(val, valSize, "%d",  commprop.dwCurrentTxQueue);
    }
	else if (epicsStrCaseCmp(key, "eventmask") == 0) {
		l = epicsSnprintf(val, valSize, "0x%X", (tty->printCharEvent ? tty->commEventMask : (tty->commEventMask & ~EV_RXCHAR)));
	}
	else {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                "Unsupported key \"%s\"", key);
        return asynError;
    }
    if (l >= valSize) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                            "Value buffer for key '%s' is too small.", key);
        return asynError;
    }
    asynPrint(tty->pasynUser, ASYN_TRACEIO_DRIVER,
              "%s getOption, key=%s, val=%s\n",
              tty->serialDeviceName, key, val);
    return asynSuccess;
}

static asynStatus
setOption(void *drvPvt, asynUser *pasynUser, const char *key, const char *val)
{
    ttyController_t *tty = (ttyController_t *)drvPvt;
    DWORD commConfigSize = sizeof(tty->commConfig);
    BOOL ret;
    DWORD error;
    COMMPROP commprop;

    assert(tty);
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
                    "%s setOption key %s val %s\n", tty->portName, key, val);
    if (tty->commHandle == INVALID_HANDLE_VALUE) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s disconnected:", tty->serialDeviceName);
        return asynError;
    }
    ret = GetCommConfig(tty->commHandle, &tty->commConfig, &commConfigSize);
    if (ret == 0) {
        error = GetLastError();
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                            "%s error calling GetCommConfig %d", tty->serialDeviceName, error);
        return asynError;
    }
    ret = GetCommProperties(tty->commHandle, &commprop);
    if (ret == 0) {
        error = GetLastError();
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                            "%s error calling GetCommProperties %d", tty->serialDeviceName, error);
        return asynError;
    }

    if (epicsStrCaseCmp(key, "baud") == 0) {
        int baud;
        if(sscanf(val, "%d", &baud) != 1) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                                                                "Bad number");
            return asynError;
        }
        tty->commConfig.dcb.BaudRate = baud;
    }
    else if (epicsStrCaseCmp(key, "bits") == 0) {
        int bits;
        if(sscanf(val, "%d", &bits) != 1) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                                                                "Bad number");
            return asynError;
        }
        tty->commConfig.dcb.ByteSize = bits;
    }
    else if (epicsStrCaseCmp(key, "parity") == 0) {
        if (epicsStrCaseCmp(val, "none") == 0) {
            tty->commConfig.dcb.Parity = 0;
        }
        else if (epicsStrCaseCmp(val, "odd") == 0) {
            tty->commConfig.dcb.Parity = 1;
        }
        else if (epicsStrCaseCmp(val, "even") == 0) {
            tty->commConfig.dcb.Parity = 2;
        }
        else if (epicsStrCaseCmp(val, "mark") == 0) {
            tty->commConfig.dcb.Parity = 3;
        }
        else if (epicsStrCaseCmp(val, "space") == 0) {
            tty->commConfig.dcb.Parity = 4;
        }
        else {
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                            "Invalid parity.");
            return asynError;
        }
    }
    else if (epicsStrCaseCmp(key, "stop") == 0) {
        if (epicsStrCaseCmp(val, "1") == 0) {
            tty->commConfig.dcb.StopBits = ONESTOPBIT;
        }
        else if (epicsStrCaseCmp(val, "1.5") == 0) {
            tty->commConfig.dcb.StopBits = ONE5STOPBITS;
        }
        else if (epicsStrCaseCmp(val, "2") == 0) {
            tty->commConfig.dcb.StopBits = TWOSTOPBITS;
        }
        else {
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                "Invalid number of stop bits.");
            return asynError;
        }
    }
    else if (epicsStrCaseCmp(key, "clocal") == 0) {
        if (epicsStrCaseCmp(val, "Y") == 0) {
            tty->commConfig.dcb.fOutxDsrFlow = FALSE;
            tty->commConfig.dcb.fDsrSensitivity = FALSE;
            tty->commConfig.dcb.fDtrControl = DTR_CONTROL_ENABLE;
        }
        else if (epicsStrCaseCmp(val, "N") == 0) {
            tty->commConfig.dcb.fOutxDsrFlow = TRUE;
            tty->commConfig.dcb.fDsrSensitivity = TRUE;
            tty->commConfig.dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;
        }
        else if (epicsStrCaseCmp(val, "D") == 0) {
            tty->commConfig.dcb.fOutxDsrFlow = FALSE;
            tty->commConfig.dcb.fDsrSensitivity = FALSE;
            tty->commConfig.dcb.fDtrControl = DTR_CONTROL_DISABLE;
        }
        else {
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                    "Invalid clocal value.");
            return asynError;
        }
    }
    else if (epicsStrCaseCmp(key, "crtscts") == 0) {
        if (epicsStrCaseCmp(val, "Y") == 0) {
            tty->commConfig.dcb.fOutxCtsFlow = TRUE;
            tty->commConfig.dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
        }
        else if (epicsStrCaseCmp(val, "N") == 0) {
            tty->commConfig.dcb.fOutxCtsFlow = FALSE;
            tty->commConfig.dcb.fRtsControl = RTS_CONTROL_ENABLE;
        }
        else if (epicsStrCaseCmp(val, "D") == 0) {
            tty->commConfig.dcb.fOutxCtsFlow = FALSE;
            tty->commConfig.dcb.fRtsControl = RTS_CONTROL_DISABLE;
        }
        else {
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                      "Invalid crtscts value.");
            return asynError;
        }
    }
    else if (epicsStrCaseCmp(key, "ixon") == 0) {
        if (epicsStrCaseCmp(val, "Y") == 0) {
            tty->commConfig.dcb.fOutX = TRUE  ;
        }
        else if (epicsStrCaseCmp(val, "N") == 0) {
            tty->commConfig.dcb.fOutX = FALSE;
        }
        else {
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                    "Invalid ixon value.");
            return asynError;
        }
    }
    else if (epicsStrCaseCmp(key, "ixany") == 0) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                    "Option ixany not supported on Windows");
        return asynError;       
    }
    else if (epicsStrCaseCmp(key, "ixoff") == 0) {
        if (epicsStrCaseCmp(val, "Y") == 0) {
            tty->commConfig.dcb.fInX = TRUE;
        }
        else if (epicsStrCaseCmp(val, "N") == 0) {
            tty->commConfig.dcb.fInX = FALSE;
        }
        else {
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                    "Invalid ixoff value.");
            return asynError;
        }
    }
    else if (epicsStrCaseCmp(key, "break_duration") == 0) {
        unsigned break_duration;
        if(sscanf(val, "%u", &break_duration) != 1) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                                                                "Bad number");
            return asynError;
        }
        tty->break_duration = break_duration;
        return asynSuccess;
    }
    else if (epicsStrCaseCmp(key, "break_delay") == 0) {
        unsigned break_delay;
        if(sscanf(val, "%u", &break_delay) != 1) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                                                                "Bad number");
            return asynError;
        }
        tty->break_delay = break_delay;
        return asynSuccess;
    }
    else if (epicsStrCaseCmp(key, "wbuff") == 0) {
        int buflen;
        if(sscanf(val, "%d", &buflen) != 1) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                                                                "Bad number");
            return asynError;
        }
        if (SetupComm(tty->commHandle, commprop.dwCurrentRxQueue, buflen) == 0)
        {
            error = GetLastError();
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                            "%s error calling SetupComm %d", tty->serialDeviceName, error);
            return asynError;
        }
    }
    else if (epicsStrCaseCmp(key, "rbuff") == 0) {
        int buflen;
        if(sscanf(val, "%d", &buflen) != 1) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                                                                "Bad number");
            return asynError;
        }
        if (SetupComm(tty->commHandle, buflen, commprop.dwCurrentTxQueue) == 0)
        {
            error = GetLastError();
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                            "%s error calling SetupComm %d", tty->serialDeviceName, error);
            return asynError;
        }
    }
    else if (epicsStrCaseCmp(key, "flush") == 0) {
        if (epicsStrCaseCmp(val, "Y") == 0) {
			tty->flush_on_write = 1;
        }
        else if (epicsStrCaseCmp(val, "N") == 0) {
			tty->flush_on_write = 0;
        }
        else {
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                      "Invalid flush value.");
            return asynError;
        }
    }
	else if (epicsStrCaseCmp(key, "eventmask") == 0) {
		int mask;
		if (sscanf(val, "%x", &mask) != 1) {
			epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
				"Bad number");
			return asynError;
		}
		if (mask & EV_RXCHAR)
		{
			tty->printCharEvent = 1;
		}
		else
		{
			tty->printCharEvent = 0;
			mask |= EV_RXCHAR; /* we always need to enable RXCHAR for internal use */
		}
		if (SetCommMask(tty->commHandle, mask) == 0)
		{
			error = GetLastError();
			epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
				"%s error calling SetCommMask %d", tty->serialDeviceName, error);
			return asynError;
		}
		tty->commEventMask = mask;
	}
	else if (epicsStrCaseCmp(key, "purge") == 0) {
	    DWORD func = 0;
		if (epicsStrCaseCmp(val, "rxabort") == 0) {
			func = PURGE_RXABORT;
		} else if (epicsStrCaseCmp(val, "txabort") == 0) {
			func = PURGE_TXABORT;
		} else if (epicsStrCaseCmp(val, "rxclear") == 0) {
			func = PURGE_RXCLEAR;
		} else if (epicsStrCaseCmp(val, "txclear") == 0) {
			func = PURGE_TXCLEAR;
		} else {
			epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
				"Invalid purge value.");
			return asynError;
		}
		if (PurgeComm(tty->commHandle, func) == 0)
		{
			error = GetLastError();
			epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
				"%s error calling PurgeComm %d", tty->serialDeviceName, error);
			return asynError;
		}
	}
	else if (epicsStrCaseCmp(key, "escape") == 0) {
	    DWORD func = 0;
		if (epicsStrCaseCmp(val, "clrbreak") == 0) {
			func = CLRBREAK;
		} else if (epicsStrCaseCmp(val, "clrdtr") == 0) {
			func = CLRDTR;
		} else if (epicsStrCaseCmp(val, "clrrts") == 0) {
			func = CLRRTS;
		} else if (epicsStrCaseCmp(val, "setbreak") == 0) {
			func = SETBREAK;
		} else if (epicsStrCaseCmp(val, "setdtr") == 0) {
			func = SETDTR;
		} else if (epicsStrCaseCmp(val, "setrts") == 0) {
			func = SETRTS;
		} else if (epicsStrCaseCmp(val, "setxoff") == 0) {
			func = SETXOFF;
		} else if (epicsStrCaseCmp(val, "setxon") == 0) {
			func = SETXON;
		} else {
			epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
				"Invalid escape value.");
			return asynError;
		}
		if (EscapeCommFunction(tty->commHandle, func) == 0)
		{
			error = GetLastError();
			epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
				"%s error calling EscapeComm %d", tty->serialDeviceName, error);
			return asynError;
		}
	}
	else if (epicsStrCaseCmp(key, "") != 0) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                                "Unsupported key \"%s\"", key);
        return asynError;
    }
    ret = SetCommConfig(tty->commHandle, &tty->commConfig, commConfigSize);
    if (ret == 0) {
        error = GetLastError();
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                            "%s error calling SetCommConfig %d", tty->serialDeviceName, error);
        return asynError;
    }
    return asynSuccess;
}
static const struct asynOption asynOptionMethods = { setOption, getOption };

/*
 * Close a connection
 */
static void
closeConnection(asynUser *pasynUser,ttyController_t *tty)
{
    if (tty->commHandle != INVALID_HANDLE_VALUE) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                           "Close %s connection.\n", tty->serialDeviceName);
		SetCommMask(tty->commHandle, 0x0); /* to exit monitorComEvents thread */
		epicsThreadSleep(0.1); /* wait for monitorComEvents to exit */
        CloseHandle(tty->commHandle);
        tty->commHandle = INVALID_HANDLE_VALUE;
        pasynManager->exceptionDisconnect(pasynUser);
    }
}

/*
 * Unblock the I/O operation
 */
static void
timeoutHandler(void *p)
{
    ttyController_t *tty = (ttyController_t *)p;

    asynPrint(tty->pasynUser, ASYN_TRACE_FLOW,
                               "%s timeout handler.\n", tty->serialDeviceName);
    tty->timeoutFlag = 1;
}


/*
 * Report link parameters
 */
static void
report(void *drvPvt, FILE *fp, int details)
{
    static const char* dtr_flow[] = { "disable", "enable", "handshake" };
    static const char* rts_flow[] = { "disable", "enable", "handshake", "toggle" };
    static const char* parity_options[] = { "none", "odd", "even", "mark", "space" };
    ttyController_t *tty = (ttyController_t *)drvPvt;
    DWORD commConfigSize = sizeof(tty->commConfig);
    DWORD modem_stat, comm_mask, error;
    BOOL ret;
    COMSTAT cstat;
    COMMPROP commprop;
    DCB dcb;

    assert(tty);
    fprintf(fp, "Serial line %s: %sonnected\n",
        tty->serialDeviceName,
        tty->commHandle != INVALID_HANDLE_VALUE ? "C" : "Disc");
    if (details >= 1) {
        fprintf(fp, "            commHandle: %p\n",  tty->commHandle);
        fprintf(fp, "    Characters written: %lu\n", tty->nWritten);
        fprintf(fp, "       Characters read: %lu\n", tty->nRead);
        fprintf(fp, "   break duration (ms): %u\n", tty->break_duration);
        fprintf(fp, "      break delay (ms): %u\n", tty->break_delay);
    }
    if (tty->commHandle != INVALID_HANDLE_VALUE && details >= 2) {
        ret = GetCommConfig(tty->commHandle, &tty->commConfig, &commConfigSize);
        if (ret == 0) {
            error = GetLastError();
            fprintf(fp, "%s error calling GetCommConfig() %d\n", tty->serialDeviceName, error);
            return;
        }
        fprintf(fp, "*** Port Configuration (DCB from GetCommConfig()) ***\n");
        fprintf(fp, "             Baud rate: %d\n", tty->commConfig.dcb.BaudRate);
        fprintf(fp, "             data bits: %d\n", (int)tty->commConfig.dcb.ByteSize);
        fprintf(fp, "                Parity: %s\n", parity_options[tty->commConfig.dcb.Parity]);
        fprintf(fp, "             stop bits: %.3g\n", 1.0 + tty->commConfig.dcb.StopBits / 2.0);
        fprintf(fp, "       Parity checking: %c\n", tty->commConfig.dcb.fParity == TRUE ? 'Y' : 'N');
        fprintf(fp, "               * Hardware flow control *\n");
        fprintf(fp, "  Out CTS flow control: %c\n", tty->commConfig.dcb.fOutxCtsFlow == TRUE ? 'Y' : 'N');
        fprintf(fp, "      RTS flow control: %s\n", rts_flow[tty->commConfig.dcb.fRtsControl]);
        fprintf(fp, "  Out DSR flow control: %c\n", tty->commConfig.dcb.fOutxDsrFlow == TRUE ? 'Y' : 'N');
        fprintf(fp, "      DTR flow control: %s\n", dtr_flow[tty->commConfig.dcb.fDtrControl]);
        fprintf(fp, "         DSR sensitive: %c\n", tty->commConfig.dcb.fDsrSensitivity == TRUE ? 'Y' : 'N');
        fprintf(fp, "              * Software flow control *\n");
        fprintf(fp, " XON/XOFF transmission: %c\n", tty->commConfig.dcb.fOutX == TRUE ? 'Y' : 'N');
        fprintf(fp, "    XON/XOFF reception: %c\n", tty->commConfig.dcb.fInX == TRUE ? 'Y' : 'N');
        fprintf(fp, "         xon char code: 0x%x\n", (int)tty->commConfig.dcb.XonChar);
        fprintf(fp, "        xoff char code: 0x%x\n", (int)tty->commConfig.dcb.XoffChar);
        fprintf(fp, "   xon character limit: %d\n", (int)tty->commConfig.dcb.XonLim);
        fprintf(fp, "  xoff character limit: %d\n", (int)tty->commConfig.dcb.XoffLim);
        fprintf(fp, "     fTXContinueOnXoff: %c\n", tty->commConfig.dcb.fTXContinueOnXoff == TRUE ? 'Y' : 'N');
        fprintf(fp, "                 * Other Settings *\n");
        fprintf(fp, "  parity -> error char: %c\n", tty->commConfig.dcb.fErrorChar == TRUE ? 'Y' : 'N');
        fprintf(fp, "    discard NULL bytes: %c\n", tty->commConfig.dcb.fNull == TRUE ? 'Y' : 'N');
        fprintf(fp, "    abort R/W on error: %c\n", tty->commConfig.dcb.fAbortOnError == TRUE ? 'Y' : 'N');
        fprintf(fp, "       error char code: 0x%x\n", (int)tty->commConfig.dcb.ErrorChar);
        fprintf(fp, "         eof char code: 0x%x\n", (int)tty->commConfig.dcb.EofChar);
        fprintf(fp, "       event char code: 0x%x\n", (int)tty->commConfig.dcb.EvtChar);
        fprintf(fp, "*** Port Properties (COMMPROP from GetCommProperties()) ***\n");
        ret = GetCommProperties(tty->commHandle, &commprop);
        if (ret == 0) {
            error = GetLastError();
            fprintf(fp, "%s error calling GetCommProperties() %d\n", tty->serialDeviceName, error);
            return;
        }
        fprintf(fp, " Max input buffer size: %d\n", (int)commprop.dwMaxRxQueue);
        fprintf(fp, "     input buffer size: %d\n", (int)commprop.dwCurrentRxQueue);
        fprintf(fp, "Max output buffer size: %d\n", (int)commprop.dwMaxTxQueue);
        fprintf(fp, "    output buffer size: %d\n", (int)commprop.dwCurrentTxQueue);
        memset(&dcb, 0, sizeof(DCB));
        dcb.DCBlength = sizeof(DCB);
        ret = GetCommState(tty->commHandle, &dcb);
        if (ret == 0) {
            error = GetLastError();
            fprintf(fp, "%s error calling GetCommState() %d\n", tty->serialDeviceName, error);
            return;
        }
        if (memcmp(&dcb, &(tty->commConfig.dcb), sizeof(DCB)) != 0)
        {
            fprintf(fp, "*** WARNING: GetCommState() and GetCommConfig() disagree on DCB contents\n");
        }
        ret = ClearCommError(tty->commHandle, &error, &cstat);
        if (ret == 0) {
            error = GetLastError();
            fprintf(fp, "%s error calling ClearCommError() %d\n", tty->serialDeviceName, error);
            return;
        }
        fprintf(fp, "*** Port Status (COMSTAT from ClearCommError()) ***\n");
        if (error != 0) {
			printCOMError(tty->serialDeviceName, error, fp);
        }
		fprintf(fp, "       waiting for CTS: %c\n", cstat.fCtsHold == TRUE ? 'Y' : 'N');
        fprintf(fp, "       waiting for DSR: %c\n", cstat.fDsrHold == TRUE ? 'Y' : 'N');
        fprintf(fp, "  waiting for DCD/RLSD: %c\n", cstat.fRlsdHold == TRUE ? 'Y' : 'N');
        fprintf(fp, "  waiting as seen Xoff: %c\n", cstat.fXoffHold == TRUE ? 'Y' : 'N');
        fprintf(fp, "  waiting as sent Xoff: %c\n", cstat.fXoffSent == TRUE ? 'Y' : 'N');
        fprintf(fp, "          EOF received: %c\n", cstat.fEof == TRUE ? 'Y' : 'N');
        fprintf(fp, "                 fTxim: %c\n", cstat.fTxim == TRUE ? 'Y' : 'N');
        fprintf(fp, " nbytes in input queue: %d\n", cstat.cbInQue);
        fprintf(fp, "nbytes in output queue: %d\n", cstat.cbOutQue);
        ret = GetCommModemStatus(tty->commHandle, &modem_stat);
        if (ret == 0) {
            error = GetLastError();
            fprintf(fp, "%s error calling GetCommModemStatus() %d\n", tty->serialDeviceName, error);
            return;
        }
        fprintf(fp, "*** Modem control line Status (from GetCommModemStatus()) ***\n");
		fprintf(fp, "                   CTS: %s\n", (modem_stat & MS_CTS_ON) ? "ON" : "OFF");
        fprintf(fp, "                   DSR: %s\n", (modem_stat & MS_DSR_ON) ? "ON" : "OFF");
        fprintf(fp, "                    RI: %s\n", (modem_stat & MS_RING_ON) ? "ON" : "OFF");
        fprintf(fp, "              DCD/RLSD: %s\n", (modem_stat & MS_RLSD_ON) ? "ON" : "OFF");
        ret = GetCommMask(tty->commHandle, &comm_mask);
        if (ret == 0) {
            error = GetLastError();
            fprintf(fp, "%s error calling GetCommMask() %d\n", tty->serialDeviceName, error);
            return;
        }
        fprintf(fp, "*** Current Comm Event Mask (from GetCommMask()) = 0x%x\n", comm_mask);
        fprintf(fp, "\n");
    }
}

/*
 * Create a link
 */
static asynStatus
connectIt(void *drvPvt, asynUser *pasynUser)
{
    char* err_mess;
    ttyController_t *tty = (ttyController_t *)drvPvt;

    /*
     * Sanity check
     */
    assert(tty);
    if (tty->commHandle != INVALID_HANDLE_VALUE) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                              "%s: Link already open!", tty->serialDeviceName);
        return asynError;
    }
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
                              "Open connection to %s\n", tty->serialDeviceName);

    /*
     * Open serial line and set configuration
     * Must open in non-blocking mode in case carrier detect is not
     * present and we plan to use the line in CLOCAL mode.
     */
    tty->commHandle = CreateFile(tty->serialDeviceName,
                                 GENERIC_READ | GENERIC_WRITE,  // access (read-write) mode
                                 0,                             // share mode
                                 0,                             // pointer to security attributes
                                 OPEN_EXISTING,                 // how to create
                                 FILE_FLAG_OVERLAPPED,          // overlapped I/O
                                 0                              // handle to file with attributes to copy
                                  );
    if (tty->commHandle == INVALID_HANDLE_VALUE) {
        err_mess = getLastErrorMessage(GetLastError());
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                            "%s Can't open: %s",
                                    tty->serialDeviceName, err_mess);
        free(err_mess);
        return asynError;
    }
    /* setOption(tty, tty->pasynUser, "baud", "9600"); */
   
    ClearCommBreak(tty->commHandle); /* in case there is one leftover from an ioc termination */
	
    /*
     * Turn off non-blocking mode
     */
    if (!FlushFileBuffers(tty->commHandle))
        return asynError;

	if (SetCommMask(tty->commHandle, tty->commEventMask) == 0)
	{
		printf("cannot set comm event mask to %d\n", tty->commEventMask);
	}

	epicsThreadCreate("monitorComEvents",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)monitorComEvents, tty);

    tty->readTimeout = -1e-99;
    tty->writeTimeout = -1e-99;

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
                          "Opened connection to %s\n", tty->serialDeviceName);
    pasynManager->exceptionConnect(pasynUser);
    return asynSuccess;
}

static asynStatus
disconnect(void *drvPvt, asynUser *pasynUser)
{
    ttyController_t *tty = (ttyController_t *)drvPvt;

    assert(tty);
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
                                    "%s disconnect\n", tty->serialDeviceName);
    epicsTimerCancel(tty->timer);
    closeConnection(pasynUser,tty);
    return asynSuccess;
}


/*
 * Write to the serial line
 */
static asynStatus writeIt(void *drvPvt, asynUser *pasynUser,
    const char *data, size_t numchars,size_t *nbytesTransfered)
{
    ttyController_t *tty = (ttyController_t *)drvPvt;
    int thisWrite;
    int nleft = (int)numchars;
    int timerStarted = 0;
    BOOL ret;
    DWORD error, dwRes;
    COMMTIMEOUTS ctimeout;
    asynStatus status = asynSuccess;

    assert(tty);
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
                            "%s write.\n", tty->serialDeviceName);
    asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, data, numchars,
                            "%s write %d\n", tty->serialDeviceName, numchars);
    if (tty->commHandle == INVALID_HANDLE_VALUE) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s disconnected:", tty->serialDeviceName);
        return asynError;
    }
    if (numchars == 0) {
        *nbytesTransfered = 0;
        return asynSuccess;
    }
    if (tty->writeTimeout != pasynUser->timeout) {
        if (pasynUser->timeout >= 0) {
            ret = GetCommTimeouts(tty->commHandle, &ctimeout);
            if (ret == 0) {
                error = GetLastError();
                epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                              "Can't get \"%s\" timeout: %f, error=%d",
                              tty->serialDeviceName, pasynUser->timeout, error);
                return asynError;
            }
            if (pasynUser->timeout == 0) {
                ctimeout.WriteTotalTimeoutMultiplier  = 0;
                ctimeout.WriteTotalTimeoutConstant    = 0;
            }
            else {
                ctimeout.WriteTotalTimeoutMultiplier  = 0; 
                // set comm write timeout to be slightly longer than asyn timeout, this is so that if
                // we do wait for this time, we are sure we will also have triggered the epicsTimer 
                // by the time we exit and so not end up waiting again 
                ctimeout.WriteTotalTimeoutConstant    = (int)(pasynUser->timeout*1000.) + 100; 
            } 
            ret = SetCommTimeouts(tty->commHandle, &ctimeout);
            if (ret == 0) {
                error = GetLastError();
                epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                              "Can't set \"%s\" timeout: %f, error=%d",
                              tty->serialDeviceName, pasynUser->timeout, error);
                return asynError;
            }
            tty->writeTimeout = pasynUser->timeout;
        }
    }
    tty->timeoutFlag = 0;
    nleft = (int)numchars;
    if (tty->writeTimeout > 0)
        {
        epicsTimerStartDelay(tty->timer, tty->writeTimeout);
        timerStarted = 1;
        }
    for (;;) {
		memset(&tty->commOverlapped, 0, sizeof(OVERLAPPED));
		tty->commOverlapped.hEvent = tty->commEventHandle;
		thisWrite = 0;
        ret = WriteFile(tty->commHandle,     // handle to file to write to
                        data,     // pointer to data to write to file
                        nleft,     // number of bytes to write
                        &thisWrite, // pointer to number of bytes written
                        &tty->commOverlapped     // pointer to structure for overlapped I/O
                        );
        if (ret == 0) {
            error = GetLastError();
			if (error != ERROR_IO_PENDING)
			{
                epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s write error: %d",
                                        tty->serialDeviceName, error);
                closeConnection(pasynUser,tty);
                status = asynError;
                break;
			}
			// set timeout to longer than comms timeout, if WaitForSingleObject then times out then something is really wrong with write 
			dwRes = WaitForSingleObject(tty->commOverlapped.hEvent, (tty->writeTimeout == 0 ? INFINITE : ctimeout.WriteTotalTimeoutConstant + 1000));
			if (dwRes == WAIT_OBJECT_0)
			{
                    if (!GetOverlappedResult(tty->commHandle, &tty->commOverlapped, &thisWrite, FALSE))
					{
						error = GetLastError();
						epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s write error: %d",
                                        tty->serialDeviceName, error);
						closeConnection(pasynUser,tty);
						status = asynError;
						break;
					}
					else if (nleft != thisWrite)
					{
				        tty->timeoutFlag = 1;
					}
			}
			else
			{
				epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s write error: WaitForSingleObject %d",
                                        tty->serialDeviceName, dwRes);
				closeConnection(pasynUser,tty);
				status = asynError;
				break;
			}
        }
        if (tty->flush_on_write != 0)
        {
            FlushFileBuffers(tty->commHandle);
        }
        tty->nWritten += thisWrite;
        nleft -= thisWrite;
        if (nleft == 0)
            break;
        data += thisWrite;
        if (tty->timeoutFlag || (tty->writeTimeout == 0)) {
            status = asynTimeout;
            break;
        }
//        printf("%s write retry, %d of %d bytes left\n", tty->serialDeviceName, nleft, (int)numchars);
    }
    if (timerStarted) epicsTimerCancel(tty->timer);
    
    /* raise a serial break if requested */
    if (tty->break_duration > 0) {
        
        FlushFileBuffers(tty->commHandle); /* ensure all data transmitted prior to break */
    
        /* Sleep after sending bytes if requested */
        if (tty->break_delay > 0) {
            Sleep(tty->break_delay);
        }
        
        if ( (ret = SetCommBreak(tty->commHandle)) == 0 ) {
            error = GetLastError();
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s SetCommBreak error: %d",
                                        tty->serialDeviceName, error);
            closeConnection(pasynUser,tty);
            status = asynError;
        } else {
            Sleep(tty->break_duration); /* wait while break is being asserted */
            if ( (ret = ClearCommBreak(tty->commHandle)) == 0 ) {
                error = GetLastError();
                epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s ClearCommBreak error: %d",
                                        tty->serialDeviceName, error);
                closeConnection(pasynUser,tty);
                status = asynError;
            }
        }
    }
    *nbytesTransfered = numchars - nleft;
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "wrote %lu %sto %s, return %s\n",
                                            (unsigned long)*nbytesTransfered,
                                            (tty->break_duration > 0 ? "(with BREAK) " : ""),
                                            tty->serialDeviceName,
                                            pasynManager->strStatus(status));
    return status;
}

static asynStatus ReadFileOverlapped(ttyController_t *tty, asynUser *pasynUser, void* data, DWORD nBytes, DWORD* nRead)
{
	asynStatus status = asynSuccess;
	int ret;
	DWORD error;
	memset(&tty->commOverlapped, 0, sizeof(OVERLAPPED));
	tty->commOverlapped.hEvent = tty->commEventHandle;
    ret = ReadFile(
                        tty->commHandle,  // handle of file to read
                        data,             // pointer to buffer that receives data
                        nBytes,                // number of bytes to read
                        nRead,        // pointer to number of bytes read
                        &tty->commOverlapped              // pointer to structure for overlapped
                        );
    if (ret == 0) {
        error = GetLastError();
		if (error != ERROR_IO_PENDING)
		{
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                      "%s read error: %d",
                      tty->serialDeviceName, error);
			closeConnection(pasynUser, tty);
			status = asynError;
		}
		else
		{
			/* set timeout to longer than comms timeout, if WaitForSingleObject then times out then something is really wrong with read */
			DWORD dwRes = WaitForSingleObject(tty->commOverlapped.hEvent, (int)(tty->readTimeout * 1000.) + 1000);
			if (dwRes == WAIT_OBJECT_0)
			{
                    if (!GetOverlappedResult(tty->commHandle, &tty->commOverlapped, nRead, FALSE))
					{
						error = GetLastError();
						epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s read error: %d",
                                        tty->serialDeviceName, error);
						closeConnection(pasynUser,tty);
						status = asynError;
					}
					else if (*nRead != nBytes)
					{
						status = asynTimeout;
					}
			}
			else
			{
				epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s read error: WaitForSingleObject %d",
                                        tty->serialDeviceName, dwRes);
				closeConnection(pasynUser,tty);
				status = asynError;
			}
        }
	}
	return status;
}

/*
 * Read from the serial line
 */
static asynStatus readIt(void *drvPvt, asynUser *pasynUser,
    char *data, size_t maxchars,size_t *nbytesTransfered,int *gotEom)
{
    ttyController_t *tty = (ttyController_t *)drvPvt;
    int thisRead, navail;
    int nRead = 0;
    COMMTIMEOUTS ctimeout;
    COMSTAT cstat;
    BOOL ret;
    DWORD error, nToRead;
    asynStatus status = asynSuccess;

    assert(tty);
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
               "%s read.\n", tty->serialDeviceName);
    if (tty->commHandle == INVALID_HANDLE_VALUE) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s disconnected:", tty->serialDeviceName);
        return asynError;
    }
    if (maxchars <= 0) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
            "%s maxchars %d Why <=0?",tty->serialDeviceName,(int)maxchars);
        return asynError;
    }
    if (tty->readTimeout != pasynUser->timeout) {
        if (pasynUser->timeout >= 0) {
            ret = GetCommTimeouts(tty->commHandle, &ctimeout);
            if (ret == 0) {
                error = GetLastError();
                epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                              "Can't get \"%s\" timeout: %f, error=%d",
                              tty->serialDeviceName, pasynUser->timeout, error);
                return asynError;
            }
            if (pasynUser->timeout == 0) {  // this is non-blocking mode, return immediately with bytes available (with zero available allowed)
                ctimeout.ReadIntervalTimeout          = MAXDWORD; 
                ctimeout.ReadTotalTimeoutMultiplier   = 0; 
                ctimeout.ReadTotalTimeoutConstant     = 0; 
            }
            else {
                ctimeout.ReadIntervalTimeout          = 0; // (in ms) return with data so far if no new data after this time once data starts being received 
                ctimeout.ReadTotalTimeoutMultiplier   = 0; 
                ctimeout.ReadTotalTimeoutConstant     = (int)(pasynUser->timeout*1000.);
            } 
            ret = SetCommTimeouts(tty->commHandle, &ctimeout);
            if (ret == 0) {
                error = GetLastError();
                epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                              "Can't set \"%s\" timeout: %f, error=%d",
                              tty->serialDeviceName, pasynUser->timeout, error);
                return asynError;
            }
            tty->readTimeout = pasynUser->timeout;
        }
    }
    tty->timeoutFlag = 0;
    if (gotEom) *gotEom = 0;
    nRead = thisRead = 0;
	status = asynTimeout;
    // we try and read one character with above timeout, and then read anything else that is also present
	navail = waitForBytes(tty, pasynUser, tty->readTimeout, 1);
	if (navail < 0)
	{
		status = asynError; /* pasynUser->errorMessage set in waitForBytes() */
	}
	if (navail > 0)
	{
		status = ReadFileOverlapped(tty, pasynUser, data, (DWORD)1, &thisRead);
	}
	if (status != asynError) {
        nRead = thisRead;
        ret = ClearCommError(tty->commHandle, &error, &cstat); // to get bytes available
        if (ret == 0) {
                error = GetLastError();
                epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                              "Can't clear \"%s\" error=%d",
                              tty->serialDeviceName, error);
                status = asynError;
        } else {
			if (error != 0)
			{
				printCOMError(tty->serialDeviceName, error, stdout);
			}
            if (cstat.cbInQue > 0) {
                nToRead = (cstat.cbInQue < maxchars - nRead ? cstat.cbInQue : (DWORD)maxchars - nRead);
                // we haven't reset CommTimeout, but as we know how many bytes are there we should return immediately
				status = ReadFileOverlapped(tty, pasynUser, data + nRead, (DWORD)nToRead, &thisRead);
                if (status == asynError) {
					; /* error message already logged */
                } else if (thisRead != nToRead) {
//                    printf("%s only read %d of %d available bytes\n",
//                          tty->serialDeviceName, thisRead, nToRead); // for debugging
                    nRead += thisRead;
                } else {
                    nRead += thisRead;
                }
            }
        }
    }
    if (nRead > 0) {
            asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, data, nRead,
                       "%s read %d\n", tty->serialDeviceName, nRead);
            tty->nRead += nRead;
    } else if (status == asynSuccess) {
        status = asynTimeout; // is we read no bytes and didn't get an error, we must have timed out
    }
    *nbytesTransfered = nRead;
    /* If there is room add a null byte */
    if (nRead < (int)maxchars)
        data[nRead] = 0;
    else if (gotEom)
        *gotEom = ASYN_EOM_CNT;
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s read %d, return %d\n",
                            tty->serialDeviceName, *nbytesTransfered, status);
    return status;
}

/*
 * Flush pending input
 */
static asynStatus
flushIt(void *drvPvt,asynUser *pasynUser)
{
    ttyController_t *tty = (ttyController_t *)drvPvt;
    BOOL ret;
    DWORD error;

    assert(tty);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s flush\n", tty->serialDeviceName);
    if (tty->commHandle == INVALID_HANDLE_VALUE) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                                "%s disconnected:", tty->serialDeviceName);
        return asynError;
    }
    ret = PurgeComm(tty->commHandle, PURGE_RXCLEAR);
    if (ret == 0) {
        error = GetLastError();
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                      "%s flush error: %d",
                      tty->serialDeviceName, error);
        return asynError;
    }
    return asynSuccess;
}

static asynOctet asynOctetMethods = { writeIt, readIt, flushIt };

/*
 * Clean up a ttyController
 */
static void
ttyCleanup(ttyController_t *tty)
{
    if (tty) {
		if (tty->commHandle != INVALID_HANDLE_VALUE)
		{
			CloseHandle(tty->commHandle);
		}
		CloseHandle(tty->commEventHandle);
		CloseHandle(tty->commEventMaskHandle);
		CloseHandle(tty->bytesAvailableEvent);
		free(tty->portName);
        free(tty->serialDeviceName);
        free(tty);
    }
}

/*
 * asynCommon methods
 */
static const struct asynCommon asynCommonMethods = {
    report,
    connectIt,
    disconnect
};

/*
 * Configure and register a generic serial device
 */
epicsShareFunc int
drvAsynSerialPortConfigure(const char *portName,
                     const char *ttyName,
                     unsigned int priority,
                     int noAutoConnect,
                     int noProcessEos)
{
    ttyController_t *tty;
    asynStatus status;
    char winTtyName[MAX_PATH];


    /*
     * Check arguments
     */
    if (portName == NULL) {
        printf("Port name missing.\n");
        return -1;
    }
    if (ttyName == NULL) {
        printf("TTY name missing.\n");
        return -1;
    }

    if(!pserialBase) serialBaseInit();
    /*
     * Create a driver
     */
    tty = (ttyController_t *)callocMustSucceed(1, sizeof(*tty), "drvAsynSerialPortConfigure()");

    /*
     * Create timeout mechanism
     */
     tty->timer = epicsTimerQueueCreateTimer(
         pserialBase->timerQueue, timeoutHandler, tty);
     if(!tty->timer) {
        printf("drvAsynSerialPortConfigure: Can't create timer.\n");
        return -1;
    }
    tty->commHandle = INVALID_HANDLE_VALUE;
    if ( (epicsStrnCaseCmp(ttyName, "\\\\.\\", 4) != 0)) {
        /* 
         * The user did not pass a Windows device name, so prepend \\.\
         */
        epicsSnprintf(winTtyName, sizeof(winTtyName), "\\\\.\\%s", ttyName);
    } 
    else {
        strncpy(winTtyName, ttyName, sizeof(winTtyName));
    }   
    tty->serialDeviceName = epicsStrDup(winTtyName);
    tty->portName = epicsStrDup(portName);
    tty->flush_on_write = 0;

	tty->commEventHandle = CreateEvent(NULL, TRUE, FALSE, NULL);
	tty->commEventMaskHandle = CreateEvent(NULL, TRUE, FALSE, NULL);
	tty->bytesAvailableEvent = CreateEvent(NULL, FALSE, FALSE, NULL); // auto reset event
	tty->commEventMask = EV_ERR | EV_RXCHAR;

    /*
     *  Link with higher level routines
     */
    tty->common.interfaceType = asynCommonType;
    tty->common.pinterface  = (void *)&asynCommonMethods;
    tty->common.drvPvt = tty;
    tty->option.interfaceType = asynOptionType;
    tty->option.pinterface  = (void *)&asynOptionMethods;
    tty->option.drvPvt = tty;
    if (pasynManager->registerPort(tty->portName,
                                   ASYN_CANBLOCK,
                                   !noAutoConnect,
                                   priority,
                                   0) != asynSuccess) {
        printf("drvAsynSerialPortConfigure: Can't register myself.\n");
        ttyCleanup(tty);
        return -1;
    }
    status = pasynManager->registerInterface(tty->portName,&tty->common);
    if(status != asynSuccess) {
        printf("drvAsynSerialPortConfigure: Can't register common.\n");
        ttyCleanup(tty);
        return -1;
    }
    status = pasynManager->registerInterface(tty->portName,&tty->option);
    if(status != asynSuccess) {
        printf("drvAsynSerialPortConfigure: Can't register option.\n");
        ttyCleanup(tty);
        return -1;
    }
    tty->octet.interfaceType = asynOctetType;
    tty->octet.pinterface  = &asynOctetMethods;
    tty->octet.drvPvt = tty;
    status = pasynOctetBase->initialize(tty->portName,&tty->octet,
                             (noProcessEos ? 0 : 1),(noProcessEos ? 0 : 1),1);
    if(status != asynSuccess) {
        printf("drvAsynSerialPortConfigure: Can't register octet.\n");
        ttyCleanup(tty);
        return -1;
    }
    tty->pasynUser = pasynManager->createAsynUser(0,0);
    status = pasynManager->connectDevice(tty->pasynUser,tty->portName,-1);
    if(status != asynSuccess) {
        printf("connectDevice failed %s\n",tty->pasynUser->errorMessage);
        ttyCleanup(tty);
        return -1;
    }
    return 0;
}

/*
 * IOC shell command registration
 */
static const iocshArg drvAsynSerialPortConfigureArg0 = { "port name",iocshArgString};
static const iocshArg drvAsynSerialPortConfigureArg1 = { "tty name",iocshArgString};
static const iocshArg drvAsynSerialPortConfigureArg2 = { "priority",iocshArgInt};
static const iocshArg drvAsynSerialPortConfigureArg3 = { "disable auto-connect",iocshArgInt};
static const iocshArg drvAsynSerialPortConfigureArg4 = { "noProcessEos",iocshArgInt};
static const iocshArg *drvAsynSerialPortConfigureArgs[] = {
    &drvAsynSerialPortConfigureArg0, &drvAsynSerialPortConfigureArg1,
    &drvAsynSerialPortConfigureArg2, &drvAsynSerialPortConfigureArg3,
    &drvAsynSerialPortConfigureArg4};
static const iocshFuncDef drvAsynSerialPortConfigureFuncDef =
                      {"drvAsynSerialPortConfigure",5,drvAsynSerialPortConfigureArgs};
static void drvAsynSerialPortConfigureCallFunc(const iocshArgBuf *args)
{
    drvAsynSerialPortConfigure(args[0].sval, args[1].sval, args[2].ival,
                               args[3].ival, args[4].ival);
}

/*
 * This routine is called before multitasking has started, so there's
 * no race condition in the test/set of firstTime.
 */
static void
drvAsynSerialPortRegisterCommands(void)
{
    static int firstTime = 1;
    if (firstTime) {
        iocshRegister(&drvAsynSerialPortConfigureFuncDef,drvAsynSerialPortConfigureCallFunc);
        firstTime = 0;
    }
}
epicsExportRegistrar(drvAsynSerialPortRegisterCommands);
