TOP=..

include $(TOP)/configure/CONFIG

#==================================================
# build a support library

LIBRARY_IOC += devAB300
devAB300_SRCS += devAB300

devAB300_LIBS += asyn
devAB300_LIBS += $(EPICS_BASE_IOC_LIBS)

DBD += devAB300.dbd
DB  += devAB300.db

include $(TOP)/configure/RULES
#----------------------------------------
