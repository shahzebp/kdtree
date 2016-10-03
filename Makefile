CC = g++

CFLAGS  = -std=c++11 -g -O0

TARGET1 = build_kdtree
TARGET2 = query_kdtree

all: $(TARGET1) $(TARGET2)

$(TARGET1):
	$(CC) $(CFLAGS) -o $(TARGET1) $(TARGET1).cpp

$(TARGET2):
	$(CC) $(CFLAGS) -o $(TARGET2) $(TARGET2).cpp

clean:
	$(RM) $(TARGET1) $(TARGET2)
