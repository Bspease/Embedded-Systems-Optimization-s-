#Target is the main .cpp file and is also the name of the exe
TARGET = videoplayer

CC=g++
CFLAGS= -mfpu=neon -Wall -std=c++11 -O0 -Werror -g -pthread
INCLUDE= -I/usr/include
LIB= -L/usr/lib/x86_64-linux-gnu -lpapi
CVDEPS = `pkg-config --cflags --libs opencv`
DEPS = ${WILDCARD *.hpp}
OBJ = $(patsubst %.cpp,%.o,$(wildcard *.cpp))

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $(INCLUDE) $(OBJ) -o $@ $(LIB) $(CVDEPS)

%.o : %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $< $(CVDEPS)

clean:
	rm -f *.o $(TARGET)
