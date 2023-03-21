# Makefile contents here
PROGRAM := search_tree
SRCS := $(wildcard *.cpp)
OBJS := ${SRCS:.cpp=.o}
CFLAGS := -g
CC = g++

test: clean debug_compile
	gdb ./$(PROGRAM)

dijkstra: debug_compile
	./$(PROGRAM) < dijkstra-test.txt

mst: debug_compile
	./$(PROGRAM) < mst-test.txt

mst1: debug_compile
	./$(PROGRAM) < mst-test1.txt

$(PROGRAM): $(SRCS)
	$(CC) $(SRCS) -o $(PROGRAM) $(CFLAGS) -O3

debug_compile: $(SRCS)
	$(CC) $(SRCS) -o $(PROGRAM) $(CFLAGS)
	
clean:
	rm -f $(PROGRAM)
	