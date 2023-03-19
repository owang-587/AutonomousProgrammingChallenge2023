# Makefile contents here
PROGRAM := dist_lookup
SRCS := $(wildcard *.cpp)
OBJS := ${SRCS:.cpp=.o}
CFLAGS := -g
CC = g++

test: clean debug_compile
	gdb ./$(PROGRAM)

dijkstra: clean debug_compile
	./$(PROGRAM) < dijkstra-test.txt

mst: clean debug_compile
	./$(PROGRAM) < mst-test.txt

$(PROGRAM): $(SRCS)
	$(CC) $(SRCS) -o $(PROGRAM) $(CFLAGS) -O3

debug_compile: clean $(SRCS)
	$(CC) $(SRCS) -o $(PROGRAM) $(CFLAGS)
	
clean:
	rm -f $(PROGRAM)
	