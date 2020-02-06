CXX = g++
CXXFLAGS = -fPIC -g -Wall -std=c++11

LIBRARY=-lEB100LEDCtrl
LIBPATH=-L../../EB100LEDCtrl/Linux

SOURCES=$(shell echo ../*.cpp)
INCLUDE=-I../../EB100LEDCtrl/Include

OBJECTS=$(SOURCES:.cpp=.o)
TARGET=EB100LEDCtrlTest

.PHONY:all clean
all:$(TARGET)

$(TARGET):$(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $(TARGET) $(LIBPATH) $(LIBRARY)

%.o:%.cpp
	$(CXX) $(CXXFLAGS) -c $^ $(INCLUDE)
	mv *.o ../

clean :
	rm $(OBJECTS) $(TARGET)

