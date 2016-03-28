# File locations
OBJDIR:=objs
HDRDIR:=../../include

DEVINC:=$(HOME)/dev/include
SYSINC:=/usr/local/include
SYSLIB:=/opt/local/lib
RLIBHDR:=$(HOME)/local_builds/libs/rlib/include
LDIR:=$(HOME)/local_builds/libs

CC:=g++
CFLAGS:=-Wall -Wno-deprecated
INCLUDES:=-I$(HDRDIR) -I$(SYSINC) -I$(DEVINC) -I$(RLIBHDR)
LFLAGS:=-ljpeg -L$(SYSLIB) -lopencv_core -lopencv_highgui -lopencv_imgproc \
	-L$(HOME)/local_builds/libs -lRFeatures \
	-L$(HOME)/local_builds/libs -lrlib

#RPATH:=-Wl,-rpath=$(LDIR):$(OPENCV)/lib
RPATH:=-Wl,-rpath=$(LDIR)

# Look for files here
vpath %.cpp src
vpath %.h $(HDRDIR)

# Application name
APP=canny

# Object files
OBJS:=$(addprefix $(OBJDIR)/, $(APP).o)

all : $(APP)

$(APP) : $(OBJS)
	$(CC) -o $@ $^ $(LFLAGS) $(RPATH)

$(OBJS) : | $(OBJDIR)

$(OBJDIR)/%.o : %.cpp %.h
	$(CC) -o $@ -c $< $(CFLAGS) $(INCLUDES)

$(OBJDIR)/%.o : %.cpp
	$(CC) -o $@ -c $< $(CFLAGS) $(INCLUDES)

$(OBJDIR) :
	mkdir $@

.PHONY : clean
clean :
	-rm -rf $(OBJDIR) $(APP)



