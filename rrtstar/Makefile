
# Compiler and Linker

CC           := gcc
LD           := gcc
#----------------------------------
#         Dependencies
#----------------------------------
# glib
CFLAGS_GLIB  := `pkg-config --cflags glib-2.0 gmodule-2.0`
LDFLAGS_GLIB := `pkg-config --libs glib-2.0 gmodule-2.0 gthread-2.0 gobject-2.0`
# geos
LDFLAGS_GEOS := -lgeos_c 
#json
LDFLAGS_JSON := -ljson
# Standard libraries
CFLAGS_STD   := -g -std=gnu99 \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter 
LDFLAGS_STD  := -lm


CFLAGS = $(CFLAGS_STD) $(CFLAGS_GLIB) $(-I//usr/local/include)
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_GLIB) $(LDFLAGS_GEOS) $(LDFLAGS_JSON)

LIB = libpath.so

PATH_OBJS =     opttree.o \
	        optsystem.o \
		optpath.o \
		kdtree.o \


PATH_DEPENDENCIES:=$(PATH_OBJS)

all: $(LIB) 
$(LIB): $(PATH_DEPENDENCIES)
	$(CC) -g -fPIC -shared -o  $@ $(PATH_OBJS) $(LDFLAGS) $(CFLAGS) 
	cp -f $@ /usr/local/lib 
	cp /home/hmaalej/Bureau/dim?/opttree.h /usr/local/include
	cp /home/hmaalej/Bureau/dim?/optsystem.h /usr/local/include
	cp /home/hmaalej/Bureau/dim?/optpath.h /usr/local/include
	cp /home/hmaalej/Bureau/dim?/kdtree.h /usr/local/include
clean:
	rm -f *.o *.a *~ $(LIB)
