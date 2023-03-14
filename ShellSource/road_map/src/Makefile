# Makefile contents here
PROGRAM := search_tree
INC_DIR = include
SRC_DIR = src
OBJ_DIR = object
SRCS := $(wildcard *.cpp)
OBJS := ${SRCS:.cpp=.o}
DEPS := $(INC_DIR)/$(wildcard *.h)
CFLAGS := -g -Wall
CC = g++

test: clean debug_compile
	gdb ./$(PROGRAM)

run: clean debug_compile
	./$(PROGRAM)

$(PROGRAM): $(SRCS)
	$(CC) $(SRCS) -o $(PROGRAM) $(CFLAGS)

debug_compile: $(SRCS) 
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/$(PROGRAM).o: $(DEPS)
	
clean:
	rm -f $(PROGRAM)

worked:
	g++ src/graph.cpp src/search_tree.cpp src/vertex.cpp -o search_tree -g -I include