#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <string>

#include <cantProceed.h>
#include <epicsAssert.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <osiSock.h>
#include <iocsh.h>

#include <epicsThread.h>
#include "asynDriver.h"
#include "asynOctet.h"
#include "asynInterposeStrip.h"
#include <epicsExport.h>

static const char *driver="stripInterpose";

struct stripPvt {
    std::string    portName;
    int addr;
    asynInterface  stripInterface;
    asynOctet      *poctet;           /* low level driver */
    void           *octetPvt;
    asynUser       *pasynUser;  /* For connect/disconnect reporting */
    std::string    stripInChars;
    std::string    stripOutChars;
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

ASYN_API int asynInterposeStripConfig(const char *portName, int addr,
                                        const char* stripInChars, const char* stripOutChars) 
{
    stripPvt     *pPvt = new stripPvt;
    asynInterface *pasynInterface;
    asynStatus    status;
    asynUser      *pasynUser;

    pPvt->portName = portName;
    pPvt->addr = addr;
    pPvt->stripInChars = (stripInChars != NULL ? stripInChars : "");
    pPvt->stripOutChars = (stripOutChars != NULL ? stripOutChars : "");
    pPvt->stripInterface.interfaceType = asynOctetType;
    pPvt->stripInterface.pinterface = &octet;
    pPvt->stripInterface.drvPvt = pPvt;
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
       &pPvt->stripInterface, &pasynInterface);
    if(status!=asynSuccess) {
        printf("%s interposeInterface failed\n", portName);
        pasynManager->freeAsynUser(pasynUser);
        delete pPvt;
        return -1;
    }
    pPvt->poctet = (asynOctet *)pasynInterface->pinterface;
    pPvt->octetPvt = pasynInterface->drvPvt;
    return(0);
}

/* asynOctet methods */
static asynStatus writeIt(void *ppvt,asynUser *pasynUser,
    const char *data,size_t numchars,size_t *nbytesTransfered)
{
    stripPvt     *pPvt = (stripPvt *)ppvt;

    if (pPvt->stripOutChars.size() == 0) {
        return pPvt->poctet->write(pPvt->octetPvt,
                pasynUser,data,numchars,nbytesTransfered);
    }
    
    // need to process properly
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s asynInterposeStrip::writeIt not implemented yet\n",
               pPvt->portName);
    *nbytesTransfered = 0;
    return asynError;
    
}

static asynStatus readIt(void *ppvt,asynUser *pasynUser,
    char *data,size_t maxchars,size_t *nbytesTransfered,int *eomReason)
{
    stripPvt *pPvt = (stripPvt *)ppvt;
    asynStatus status = pPvt->poctet->read(pPvt->octetPvt,
            pasynUser,data,maxchars,nbytesTransfered,eomReason);
    if (pPvt->stripInChars.size() == 0) {
        return status;
    }
    size_t n = 0;
    for(size_t i=0; i<*nbytesTransfered; ++i) {
        if (pPvt->stripInChars.find(data[i]) == std::string::npos) {
            data[n++] = data[i];
        }
    }
    if (n != *nbytesTransfered) {
        asynPrintIO(pasynUser,ASYN_TRACEIO_FILTER,
            data,n,"asynInterposeStrip:readIt %s stripped %llu characters\n",
               pPvt->portName, (epicsUInt64)(*nbytesTransfered - n));
        *nbytesTransfered = n;
        if (n < maxchars) {
            data[n] = 0;
        }
    }
    return status;
}

static asynStatus flushIt(void *ppvt,asynUser *pasynUser)
{
    stripPvt *pPvt = (stripPvt *)ppvt;

    return pPvt->poctet->flush(pPvt->octetPvt,pasynUser);
}

static asynStatus registerInterruptUser(void *ppvt,asynUser *pasynUser,
    interruptCallbackOctet callback, void *userPvt,void **registrarPvt)
{
    stripPvt *pPvt = (stripPvt *)ppvt;

    return pPvt->poctet->registerInterruptUser(pPvt->octetPvt,
        pasynUser,callback,userPvt,registrarPvt);
}

static asynStatus cancelInterruptUser(void *ppvt,asynUser *pasynUser,
     void *registrarPvt)
{
    stripPvt *pPvt = (stripPvt *)ppvt;

    return pPvt->poctet->cancelInterruptUser(pPvt->octetPvt,
        pasynUser,registrarPvt);
}

static asynStatus setInputEos(void *ppvt,asynUser *pasynUser,
    const char *eos,int eoslen)
{
    stripPvt *pPvt = (stripPvt *)ppvt;

    return pPvt->poctet->setInputEos(pPvt->octetPvt,pasynUser,
           eos,eoslen);
}

static asynStatus getInputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize,int *eoslen)
{
    stripPvt *pPvt = (stripPvt *)ppvt;

        return pPvt->poctet->getInputEos(pPvt->octetPvt,pasynUser,
           eos,eossize,eoslen);
}

static asynStatus setOutputEos(void *ppvt,asynUser *pasynUser,
    const char *eos, int eoslen)
{
    stripPvt *pPvt = (stripPvt *)ppvt;
    
    return pPvt->poctet->setOutputEos(pPvt->octetPvt, pasynUser, eos, eoslen);
}

static asynStatus getOutputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize,int *eoslen)
{
    stripPvt *pPvt = (stripPvt *)ppvt;
    return pPvt->poctet->getOutputEos(pPvt->octetPvt, pasynUser, eos, eossize, eoslen);
}

/* register asynInterposeStripConfig*/
static const iocshArg asynInterposeStripConfigArg0 =
    { "portName", iocshArgString };
static const iocshArg asynInterposeStripConfigArg1 =
    { "addr", iocshArgInt };
static const iocshArg asynInterposeStripConfigArg2 =
    { "stripInChars", iocshArgString };
static const iocshArg asynInterposeStripConfigArg3 =
    { "stripOutChars", iocshArgString };
static const iocshArg *asynInterposeStripConfigArgs[] =
    {&asynInterposeStripConfigArg0,&asynInterposeStripConfigArg1,
     &asynInterposeStripConfigArg2,&asynInterposeStripConfigArg3};
static const iocshFuncDef asynInterposeStripConfigFuncDef =
    {"asynInterposeStripConfig", 4, asynInterposeStripConfigArgs};
static void asynInterposeStripConfigCallFunc(const iocshArgBuf *args)
{
    asynInterposeStripConfig(args[0].sval,args[1].ival,
          args[2].sval,args[3].sval);
}

extern "C" {

static void asynInterposeStripRegister(void)
{
    static int firstTime = 1;
    if (firstTime) {
        firstTime = 0;
        iocshRegister(&asynInterposeStripConfigFuncDef, asynInterposeStripConfigCallFunc);
    }
}
epicsExportRegistrar(asynInterposeStripRegister);

}
