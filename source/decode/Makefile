MAIN = main.o
OBJS = stdafx.o
CPPFLAGS = -O3 -I$(HOME)/intel/mkl/include -I../../common -I../../levmar
LIBS = -L$(HOME)/intel/mkl/lib/ia32 -lmkl_core -lmkl_pgi_thread -lmkl_rt -lmkl_scalapack_core -lmkl_sequential

all: $(OBJS) $(MAIN)
	g++ $(OPTFLAGS) $(CPPFLAGS) $(OBJS) $(MAIN) $(LIBS)

clean:
	rm $(OBJS) $(MAIN)
