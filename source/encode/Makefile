MAIN = main.o
OBJS = stdafx.o
CPPFLAGS = -O3 -I$(HOME)/intel/mkl/include -I../../common -I../../levmar
LIBS = 

all: $(OBJS) $(MAIN)
	g++ $(OPTFLAGS) $(CPPFLAGS) $(OBJS) $(MAIN) $(LIBS)

clean:
	rm $(OBJS) $(MAIN)
