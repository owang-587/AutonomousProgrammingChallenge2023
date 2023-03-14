# Makefile contents here
PROGRAM := search_tree
SRCS := $(wildcard *.cpp)
OBJS := ${SRCS:.cpp=.o}
CFLAGS := -g -Wall 
CC = g++

test: clean debug_compile
	gdb ./$(PROGRAM)

run: clean debug_compile
	./$(PROGRAM) < dijkstra-test.txt

$(PROGRAM): $(SRCS)
	$(CC) $(SRCS) -o $(PROGRAM) $(CFLAGS) -O3

debug_compile: clean $(SRCS)
	$(CC) $(SRCS) -o $(PROGRAM) $(CFLAGS)
	
clean:
	rm -f $(PROGRAM)
	