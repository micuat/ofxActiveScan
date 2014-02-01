# All variables and this file are optional, if they are not present the PG and the
# makefiles will try to parse the correct values from the file system.
#
# Variables that specify exclusions can use % as a wildcard to specify that anything in
# that position will match. A partial path can also be specified to, for example, exclude
# a whole folder from the parsed paths from the file system
#
# Variables can be specified using = or +=
# = will clear the contents of that variable both specified from the file or the ones parsed
# from the file system
# += will add the values to the previous ones in the file or the ones parsed from the file 
# system
# 
# The PG can be used to detect errors in this file, just create a new project with this addon 
# and the PG will write to the console the kind of error and in which line it is

meta:
	ADDON_NAME = ofxActiveScan
	ADDON_DESCRIPTION = Addon for projector-camera auto-calibration and 3D reconstruction
	ADDON_AUTHOR = Naoto HIEDA
	ADDON_TAGS = "computer vision" "camera calibration" "3D reconstruction"
	ADDON_URL = http://github.com/micuat/ofxActiveScan

common:
	# dependencies with other addons, a list of them separated by spaces 
	# or use += in several lines
	# ADDON_DEPENDENCIES =
	
	# include search paths, this will be usually parsed from the file system
	# but if the addon or addon libraries need special search paths they can be
	# specified here separated by spaces or one per line using +=
	# ADDON_INCLUDES =
	
	# any special flag that should be passed to the compiler when using this
	# addon
	# ADDON_CFLAGS =
	
	# any special flag that should be passed to the linker when using this
	# addon, also used for system libraries with -lname
	# ADDON_LDFLAGS =
	
	# linux only, any library that should be included in the project using
	# pkg-config
	# ADDON_PKG_CONFIG_LIBRARIES =
	
	# osx/iOS only, any framework that should be included in the project
	# ADDON_FRAMEWORKS =
	
	# source files, these will be usually parsed from the file system looking
	# in the src folders in libs and the root of the addon. if your addon needs
	# to include files in different places or a different set of files per platform
	# they can be specified here
	# ADDON_SOURCES =
	
	# some addons need resources to be copied to the bin/data folder of the project
	# specify here any files that need to be copied, you can use wildcards like * and ?
	# ADDON_DATA = 
	
	# when parsing the file system looking for libraries exclude this for all or
	# a specific platform
	# ADDON_LIBS_EXCLUDE =
	
linux:
	# binary libraries, these will be usually parsed from the file system but some 
	# libraries need to passed to the linker in a specific order 
	ADDON_LIBS =
	ADDON_LIBS += libs/lapack/lib/linux/libblas.a
	ADDON_LIBS += libs/lapack/lib/linux/liblapack.a
	ADDON_LIBS += libs/lapack/lib/linux/liblapacke.a
	ADDON_LIBS += libs/lapack/lib/linux/libtmglib.a
	ADDON_LIBS += libs/levmar/lib/linux/liblevmar.a
	
vs:
	ADDON_CFLAGS = "/D ADD_ /D HAVE_LAPACK_CONFIG_H /D LAPACK_COMPLEX_STRUCTURE /D _USE_MATH_DEFINES"
	ADDON_LIBS =
	ADDON_LIBS += libs/lapack/lib/vs/libblas.lib
	ADDON_LIBS += libs/lapack/lib/vs/libblasd.lib
	ADDON_LIBS += libs/lapack/lib/vs/liblapack.lib
	ADDON_LIBS += libs/lapack/lib/vs/liblapackd.lib
	ADDON_LIBS += libs/lapack/lib/vs/liblapacke.lib
	ADDON_LIBS += libs/lapack/lib/vs/liblapacked.lib
	ADDON_LIBS += libs/levmar/lib/vs/liblevmar.lib
	ADDON_LIBS += libs/levmar/lib/vs/liblevmard.lib
	ADDON_DLLS_TO_COPY =
	ADDON_DLLS_TO_COPY += libs/lapack/lib/vs/libblas.dll
	ADDON_DLLS_TO_COPY += libs/lapack/lib/vs/liblapack.dll
	ADDON_DLLS_TO_COPY += libs/lapack/lib/vs/liblapacke.dll
	ADDON_DLLS_TO_COPY += libs/levmar/lib/vs/liblevmar.dll
	ADDON_DLLS_TO_COPY += libs/mingw/lib/vs/libgcc_s_dw2-1.dll
	ADDON_DLLS_TO_COPY += libs/mingw/lib/vs/libgfortran-3.dll
	ADDON_DLLS_TO_COPY += libs/mingw/lib/vs/libquadmath-0.dll

osx:
	ADDON_LIBS =
	ADDON_LIBS += libs/f2c/lib/osx/libf2c.a
	ADDON_LIBS += libs/gfortran/lib/osx/libgfortran.a
	ADDON_LIBS += libs/lapack/lib/osx/libblas.a
	ADDON_LIBS += libs/lapack/lib/osx/liblapack.a
	ADDON_LIBS += libs/lapack/lib/osx/liblapacke.a
	ADDON_LIBS += libs/lapack/lib/osx/libtmglib.a
	ADDON_LIBS += libs/levmar/lib/osx/liblevmar.a
