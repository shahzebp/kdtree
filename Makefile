# the compiler: gcc for C program, define as g++ for C++
CC = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS  = -std=c++11 -g

# the build target executable:
TARGET1 = build_kdtree
TARGET2 = query_kdtree

all: $(TARGET1) $(TARGET2)

$(TARGET1):
	$(CC) $(CFLAGS) -o $(TARGET1) $(TARGET1).cpp

$(TARGET2):
	$(CC) $(CFLAGS) -o $(TARGET2) $(TARGET2).cpp

run:
	./$(TARGET)
clean:
	$(RM) $(TARGET1) $(TARGET2) *.kd
