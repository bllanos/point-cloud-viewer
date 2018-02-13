FLAGS = -I/usr/include -I/usr/include/eigen3 -L/usr/lib -L/usr/lib/nvidia-375 -lglut -lGLEW -lGL -lGLU -lX11 -lCGAL -lgmp -lboost_system -Wno-write-strings -Wno-int-in-bool-context -Wall -std=c++11

HEADERS = error.h utilities.h

EXEC = pointCloudViewer

pointCloudViewer:	pointcloud.o pointcloudviewer.o affinetransform.o camera.o litmaterial.o point.o utilities.o
	g++ -o pointCloudViewer pointcloudviewer.o affinetransform.o camera.o litmaterial.o pointcloud.o point.o utilities.o $(FLAGS)

affinetransform.o:	affinetransform.cpp affinetransform.h $(HEADERS)
	g++ -c $(FLAGS) affinetransform.cpp -o affinetransform.o

camera.o:	camera.cpp camera.h affinetransform.h $(HEADERS)
	g++ -c $(FLAGS) camera.cpp -o camera.o

litmaterial.o:	litmaterial.cpp litmaterial.h $(HEADERS)
	g++ -c $(FLAGS) litmaterial.cpp -o litmaterial.o

point.o:	point.cpp point.h pointcloud.h $(HEADERS)
		g++ -c $(FLAGS) point.cpp -o point.o

pointcloud.o:	pointcloud.cpp pointcloud.h affinetransform.h point.h $(HEADERS)
		g++ -c $(FLAGS) pointcloud.cpp -o pointcloud.o

pointcloudviewer.o:	pointcloudviewer.cpp camera.h litmaterial.h object.h pointcloud.h $(HEADERS)
		g++ -c $(FLAGS) pointcloudviewer.cpp -o pointcloudviewer.o

utilities.o:	utilities.cpp $(HEADERS)
		g++ -c $(FLAGS) utilities.cpp -o utilities.o

all:	$(EXEC)

clean:
	rm -f *.o *.gch $(EXEC) out.csv results.csv
