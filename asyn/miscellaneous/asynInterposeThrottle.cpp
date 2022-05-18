#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>

#include <cantProceed.h>
#include <epicsAssert.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <osiSock.h>
#include <iocsh.h>

#include <epicsThread.h>
#include "asynDriver.h"
#include "asynOctet.h"
#include "asynInterposeThrottle.h"
#include <epicsExport.h>

static const char *driver="throttleInterpose";

struct throttlePvt {
    std::string    portName;
    int addr;
    asynInterface  throttleInterface;
    asynOctet      *poctet;           /* low level driver */
    void           *octetPvt;
    asynUser       *pasynUser;  /* For connect/disconnect reporting */
    double minDelay;   /* minimum delay between commands */
    epicsTimeStamp ts; /* time of last write to device */
};

/* asynOctet methods */
static asynStatus writeIt(void *ppvt,asynUser *pasynUser,
    const char *data,size_t numchars,size_t *nbytesTransfered);
static asynStatus readIt(void *ppvt,asynUser *pasynUser,
    char *data,size_t maxchars,size_t *nbytesTransfered,int *eomReason);
static asynStatus flushIt(void *ppvt,asynUser *pasynUser);
static asynStatus registerInterruptUser(void *ppvt,asynUser *pasynUser,
    interruptCallbackOctet callback, void *userPvt,void **registrarPvt);
static asynStatus cancelInterruptUser(void *drvPvt,asynUser *pasynUser,
     void *registrarPvt);
static asynStatus setInputEos(void *ppvt,asynUser *pasynUser,
    const char *eos,int eoslen);
static asynStatus getInputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize ,int *eoslen);
static asynStatus setOutputEos(void *ppvt,asynUser *pasynUser,
    const char *eos,int eoslen);
static asynStatus getOutputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize,int *eoslen);

static asynOctet octet = {
    writeIt,readIt,flushIt,
    registerInterruptUser, cancelInterruptUser,
    setInputEos,getInputEos,setOutputEos,getOutputEos
};

ASYN_API int asynInterposeThrottleConfig(const char *portName, int addr, double minDelay) 
{
    asynInterface *pasynInterface;
    asynStatus    status;
    asynUser      *pasynUser;

    if (portName == NULL) {
        printf("asynInterposeThrottleConfig: no port specified\n");
        return -1;
    }

    throttlePvt     *pPvt = new throttlePvt;
    pPvt->portName = portName;
    pPvt->addr = addr;
    pPvt->minDelay = minDelay;
    epicsTimeGetCurrent(&pPvt->ts);
    
    pPvt->throttleInterface.interfaceType = asynOctetType;
    pPvt->throttleInterface.pinterface = &octet;
    pPvt->throttleInterface.drvPvt = pPvt;
    pasynUser = pasynManager->createAsynUser(0,0);
    pPvt->pasynUser = pasynUser;
    pPvt->pasynUser->userPvt = pPvt;
    status = pasynManager->connectDevice(pasynUser,portName,addr);
    if(status!=asynSuccess) {
        printf("%s connectDevice failed\n",portName);
        pasynManager->freeAsynUser(pasynUser);
        delete pPvt;
        return -1;
    }
    /* Find the asynOctet interface */
    pasynInterface = pasynManager->findInterface(pasynUser, asynOctetType, 1);
    if (!pasynInterface) {
        printf("%s findInterface error for asynOctetType %s\n",
               portName, pasynUser->errorMessage);
        pasynManager->freeAsynUser(pasynUser);
        delete pPvt;
        return -1;
    }
    
    status = pasynManager->interposeInterface(portName, addr,
       &pPvt->throttleInterface, &pasynInterface);
    if(status!=asynSuccess) {
        printf("%s interposeInterface failed\n", portName);
        pasynManager->freeAsynUser(pasynUser);
        delete pPvt;
        return -1;
    }
    pPvt->poctet = (asynOctet *)pasynInterface->pinterface;
    pPvt->octetPvt = pasynInterface->drvPvt;
    printf("asynInterposeThrottleConfig: min write delay %f on port %s addr %d\n",
       minDelay, portName, addr);
    return(0);
}

/* asynOctet methods */
static asynStatus writeIt(void *ppvt,asynUser *pasynUser,
    const char *data,size_t numchars,size_t *nbytesTransfered)
{
    throttlePvt     *pPvt = (throttlePvt *)ppvt;
    epicsTimeStamp now;
    epicsTimeGetCurrent(&now);
    double delay = pPvt->minDelay - epicsTimeDiffInSeconds(&now, &pPvt->ts);
    if (delay > 0.0) {
        asynPrintIO(pasynUser, ASYN_TRACEIO_FILTER, data, numchars,
               "asynInterposeThrottle:writeIt %s delaying %f seconds\n",
               pPvt->portName.c_str(), delay);
        epicsThreadSleep(delay);
    }
    asynStatus status = pPvt->poctet->write(pPvt->octetPvt,
                pasynUser,data,numchars,nbytesTransfered);    
    epicsTimeGetCurrent(&pPvt->ts);
    return status;
}

static asynStatus readIt(void *ppvt,asynUser *pasynUser,
    char *data,size_t maxchars,size_t *nbytesTransfered,int *eomReason)
{
    throttlePvt *pPvt = (throttlePvt *)ppvt;

    return pPvt->poctet->read(pPvt->octetPvt,
            pasynUser,data,maxchars,nbytesTransfered,eomReason);
}

static asynStatus flushIt(void *ppvt,asynUser *pasynUser)
{
    throttlePvt *pPvt = (throttlePvt *)ppvt;

    return pPvt->poctet->flush(pPvt->octetPvt,pasynUser);
}

static asynStatus registerInterruptUser(void *ppvt,asynUser *pasynUser,
    interruptCallbackOctet callback, void *userPvt,void **registrarPvt)
{
    throttlePvt *pPvt = (throttlePvt *)ppvt;

    return pPvt->poctet->registerInterruptUser(pPvt->octetPvt,
        pasynUser,callback,userPvt,registrarPvt);
}

static asynStatus cancelInterruptUser(void *ppvt,asynUser *pasynUser,
     void *registrarPvt)
{
    throttlePvt *pPvt = (throttlePvt *)ppvt;

    return pPvt->poctet->cancelInterruptUser(pPvt->octetPvt,
        pasynUser,registrarPvt);
}

static asynStatus setInputEos(void *ppvt,asynUser *pasynUser,
    const char *eos,int eoslen)
{
    throttlePvt *pPvt = (throttlePvt *)ppvt;

    return pPvt->poctet->setInputEos(pPvt->octetPvt,pasynUser,
           eos,eoslen);
}

static asynStatus getInputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize,int *eoslen)
{
    throttlePvt *pPvt = (throttlePvt *)ppvt;

    return pPvt->poctet->getInputEos(pPvt->octetPvt,pasynUser,
           eos,eossize,eoslen);
}

static asynStatus setOutputEos(void *ppvt,asynUser *pasynUser,
    const char *eos, int eoslen)
{
    throttlePvt *pPvt = (throttlePvt *)ppvt;
    
    return pPvt->poctet->setOutputEos(pPvt->octetPvt, pasynUser, eos, eoslen);
}

static asynStatus getOutputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize,int *eoslen)
{
    throttlePvt *pPvt = (throttlePvt *)ppvt;
    return pPvt->poctet->getOutputEos(pPvt->octetPvt, pasynUser, eos, eossize, eoslen);
}

/* register asynInterposeThrottleConfig*/
static const iocshArg asynInterposeThrottleConfigArg0 =
    { "portName", iocshArgString };
static const iocshArg asynInterposeThrottleConfigArg1 =
    { "addr", iocshArgInt };
static const iocshArg asynInterposeThrottleConfigArg2 =
    { "minDelay", iocshArgDouble };
static const iocshArg *asynInterposeThrottleConfigArgs[] =
    {&asynInterposeThrottleConfigArg0,&asynInterposeThrottleConfigArg1,
     &asynInterposeThrottleConfigArg2};
static const iocshFuncDef asynInterposeThrottleConfigFuncDef =
    {"asynInterposeThrottleConfig", 3, asynInterposeThrottleConfigArgs};
static void asynInterposeThrottleConfigCallFunc(const iocshArgBuf *args)
{
    asynInterposeThrottleConfig(args[0].sval,args[1].ival,
          args[2].dval);
}

extern "C" {

static void asynInterposeThrottleRegister(void)
{
    static int firstTime = 1;
    if (firstTime) {
        firstTime = 0;
        iocshRegister(&asynInterposeThrottleConfigFuncDef, asynInterposeThrottleConfigCallFunc);
    }
}
epicsExportRegistrar(asynInterposeThrottleRegister);

}
