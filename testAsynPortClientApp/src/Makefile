TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
PROD_HOST += testAsynIPPortClient

# only build on Linux or recent Windows VisualStudio
ifneq ($(VisualStudioVersion),)
PROD_HOST += testSerialPort
endif
ifneq ($(OS_CLASS),WIN32)
PROD_HOST += testSerialPort
endif

testAsynIPPortClient_SRCS += testAsynIPPortClient.cpp

testSerialPort_SRCS += testSerialPort.cpp

testSerialPort_CXXFLAGS += -I$(CLI11)

PROD_LIBS += asyn
ifeq ($(EPICS_LIBCOM_ONLY),YES)
  PROD_LIBS += Com
else
  PROD_LIBS += $(EPICS_BASE_IOC_LIBS)
endif

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
