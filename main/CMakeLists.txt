idf_component_register(SRCS "main.c" "esp32c3_szp.c" "app_process.c" "motion_control.c"
                    INCLUDE_DIRS "")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)  
