TARGET := ALARS_Server 

PI_ENABLE=0
EKF_ENABLE=1
IMU_ENABLE=1
PID_ENABLE=1
SERVO_ENABLE=1
INV_ENABLE=1






BUILD_DIR := ./build
SRC_DIR :=./src 


LDFLAGS := -lpthread

CXX:=g++-12 
CXX_FLAGS:=-I ./src/ -DPI_ENABLE=$(PI_ENABLE) \
					 					 -DEKF_ENABLE=$(EKF_ENABLE) \
										 -DIMU_ENABLE=$(IMU_ENABLE) \
										 -DPID_ENABLE=$(PID_ENABLE) \
										 -DSERVO_ENABLE=$(SERVO_ENABLE) \
										 -DINV_ENABLE=$(INV_ENABLE)

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
				@$(CXX) $(CXX_FLAGS) -c $< -o $@ -g -std=c++20

$(TARGET): $(OBJS)
		@echo "Linking $(TARGET)"
			@$(CXX)  $(CXX_FLAGS) -o $@ $^ $(LDFLAGS)  -std=c++20
