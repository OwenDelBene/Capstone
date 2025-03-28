TARGET := ALARS_Server 

PI_ENABLE = 0
EKF_ENABLE = 0
IMU_ENABLE = 0

BUILD_DIR := ./build
SRC_DIR :=./src 


LDFLAGS := -lpthread

CXX:=g++-12 -I ./src/

SRCS:= $(shell find $(SRC_DIR) -name '*.cpp')
SRCS+= $(shell find ./src/ahrs -name '*.cpp')
OBJS := $(SRCS:src/%.cpp=$(BUILD_DIR)/%.o)
.PHONY: all
all: $(TARGET)

.PHONY: clean
clean:
		rm $(TARGET)
		rm -r $(BUILD_DIR)/*


$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
		@mkdir -p $(dir $@)
			@echo "Building $<"
				@$(CXX) -c $< -o $@ -g -std=c++20

$(TARGET): $(OBJS)
		@echo "Linking $(TARGET)"
			@$(CXX)  -o $@ $^ $(LDFLAGS)  -std=c++20
