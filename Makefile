BUILD_DIR = build
TARGET_NAME = sensor_mapper
CORES = $(shell nproc)

.PHONY: all debug release run clean

all: debug

debug:
	cmake -B $(BUILD_DIR)/debug -S . -DCMAKE_BUILD_TYPE=Debug
	cmake --build $(BUILD_DIR)/debug -- -j$(CORES)

release:
	cmake -B $(BUILD_DIR)/release -S . -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD_DIR)/release -- -j$(CORES)

run: debug
	./$(BUILD_DIR)/debug/$(TARGET_NAME)

clean:
	rm -rf $(BUILD_DIR)
