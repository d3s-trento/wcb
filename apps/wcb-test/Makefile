CONTIKI_PROJECT = wcb-test
CONTIKI_WITH_RIME = 1

DEFINES += PROJECT_CONF_H=\"project-conf.h\"

PROJECTDIRS += ../deployment
PROJECT_SOURCEFILES += deployment.c
PROJECT_SOURCEFILES += mathfunct.c

include ../../Makefile.wcb

all: $(CONTIKI_PROJECT)
CONTIKI = ../../contiki
include $(CONTIKI)/Makefile.include



