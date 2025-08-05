idf_component_register(SRCS "main.cpp" "MotionController.cpp" "UartHandler.cpp" "driver/uart.cpp" "driver/servo.cpp"
                    INCLUDE_DIRS "./" "driver")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)  
