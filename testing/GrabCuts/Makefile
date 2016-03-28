# File locations
OBJDIR:=objs
HDRDIR:=../../include

DEVINC:=${HOME}/dev/include
SYSINC:=/usr/local/include
SYSLIB:=/opt/local/lib

CC:=g++
CFLAGS:=-Wall -Wno-deprecated
INCLUDES:=-I$(HDRDIR) -I$(SYSINC) -I$(DEVINC) -I/opt/local/include
LFLAGS:=-ljpeg -L$(SYSLIB) -lopencv_core -lopencv_highgui -lopencv_imgproc \
	-L${HOME}/local_builds/rlib -lrlib -L${HOME}/local_builds/RFeatures -lRFeatures

# Look for files here
vpath %.cpp src
vpath %.h $(HDRDIR)

# Application name
APP=test

# Object files
OBJS:=$(addprefix $(OBJDIR)/, $(APP).o)

all : $(APP)

$(APP) : $(OBJS)
	$(CC) -o $@ $^ $(LFLAGS)

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
