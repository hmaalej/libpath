
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
CMD = path

PATH_OBJS =     rrtstar/opttree.o \
	        rrtstar/optsystem.o \
		rrtstar/kdtree.o \
		optpath.o \


PATH_DEPENDENCIES:=$(PATH_OBJS)

all: $(LIB) 
$(LIB): $(PATH_DEPENDENCIES)
	$(CC) -g -fPIC -shared -o $@ $(PATH_OBJS) $(LDFLAGS) $(CFLAGS) 
install:	
	cp -f $(LIB) /usr/local/lib 
	cp opttree.h /usr/local/include
	cp optsystem.h /usr/local/include
	cp optpath.h /usr/local/include
	cp kdtree.h /usr/local/include
path_main.o: path_main.c
	$(CC) $(CFLAGS_GLIB) -c path_main.c
$(CMD): path_main.o
	$(CC) -o $@ path_main.o -lpath
docs:
	doxygen Doxyfile
clean:
	rm -f *.o *.a *~ $(LIB) $(CMD) 
