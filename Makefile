# Define the compiler and compiler flags
CXX := g++
CXXFLAGS := -O2 -std=c++20

# Define the target executable and the source files
TARGET := simulation
SRCS := simulation.cpp sequence.cpp filters.cpp

# Define the object files based on the source files
OBJS := $(SRCS:.cpp=.o)

# The default rule that will be executed when you run 'make'
all: $(TARGET)

# Rule to link the object files into the final executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Rule to compile the source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to clean up the build directory
clean:
	rm -f $(TARGET) $(OBJS)

# Phony targets to avoid conflicts with files named 'all' or 'clean'
.PHONY: all clean