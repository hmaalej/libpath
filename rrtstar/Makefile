
# Compiler and Linker
CC           := gcc
LD           := gcc

# Standard libraries
CFLAGS_STD   := -g -std=gnu99 \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter 
LDFLAGS_STD  := -lm

# glib
CFLAGS_GLIB  := `pkg-config --cflags glib-2.0 gmodule-2.0`
LDFLAGS_GLIB := `pkg-config --libs glib-2.0 gmodule-2.0 gthread-2.0 gobject-2.0`

%.o: %.c %.h
	@echo "    [$@]"
	$(CC) $(CFLAGS) -c $< 


CFLAGS = $(CFLAGS_STD) $(CFLAGS_GLIB)


LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_GLIB) 


OPTTREE_OBJS =  opttree.o \
	        optsystem.o \
		optmain.o \
		kdtree.o

OPTTREE=./rrtst
OPTTREE_DEPENDENCIES:=$(OPTTREE_OBJS) $(LDFLAGS_COMMON)

all: $(OPTTREE) 


$(OPTTREE): $(OPTTREE_DEPENDENCIES)
	$(CC) -g -o $@ $(OPTTREE_OBJS) $(LDFLAGS) $(CFLAGS) 


clean:
	rm -f *.o *.a *~ $(OPTTREE) $(OPTSYSTEM_TEST)
