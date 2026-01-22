BUILD_DIR = build
TARGET_NAME = sensor_mapper

# Detect OS for executable extension
ifeq ($(OS),Windows_NT)
    EXT = .exe
else
    EXT =
endif

.PHONY: all debug release run clean

all: debug

debug:
	cmake -B $(BUILD_DIR)/debug -S . -DCMAKE_BUILD_TYPE=Debug
	cmake --build $(BUILD_DIR)/debug --parallel

release:
	cmake -B $(BUILD_DIR)/release -S . -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD_DIR)/release --parallel

run: debug
	./$(BUILD_DIR)/debug/$(TARGET_NAME)$(EXT)

clean:
	cmake -E remove_directory $(BUILD_DIR)
