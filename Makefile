OMPL_INCDIR = /home/jingru/workspace/Library/ompl-0.14.0-Source/src
OMPL_LIBDIR = /home/jingru/workspace/Library/ompl-0.14.0-Source/build/Release/lib
KRISLIBRARY_INCDIR = /home/jingru/workspace/Library/KrisLibrary/
KRISLIBRARY_LIBDIR = /home/jingru/workspace/Library/KrisLibrary/lib/

LIBS = -lompl -lKrisLibrary -linclude

benchmark:
	g++ LineSegment.cpp Linkage.cpp Polyhedron.cpp utility.cpp EnvironmentSettings.cpp BenchmarkSettings.cpp RRTOMPL.cpp RRTStarOMPL.cpp CEntropy.cpp FMM.cpp PRMStar.cpp PRMStarCSpace.cpp PlannerEntrance.cpp benchmark.cpp -I$(KRISLIBRARY_INCDIR) -L$(KRISLIBRARY_LIBDIR) -I$(OMPL_INCDIR) -L$(OMPL_LIBDIR) $(LIBS) -lboost_thread -o benchmark


clean:
	rm -rf *.o main benchmark
