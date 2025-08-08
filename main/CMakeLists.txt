idf_component_register(SRCS "main.cpp" "MotionController.cpp" "UartHandler.cpp" "driver/PCA9685.cpp"
                    INCLUDE_DIRS "./" "driver"
                    EMBED_TXTFILES "www/index.html")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)  
