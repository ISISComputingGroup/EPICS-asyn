TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
PROD_HOST += testAsynIPPortClient

testAsynIPPortClient_SRCS += testAsynIPPortClient.cpp

ifeq ($(findstring 10.0,$(VCVERSION)),)
PROD_HOST += testSerialPort
testSerialPort_SRCS += testSerialPort.cpp
testSerialPort_CXXFLAGS += -I$(CLI11)
endif

PROD_LIBS += asyn
ifeq ($(EPICS_LIBCOM_ONLY),YES)
  PROD_LIBS += Com
else
  PROD_LIBS += $(EPICS_BASE_IOC_LIBS)
endif

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
