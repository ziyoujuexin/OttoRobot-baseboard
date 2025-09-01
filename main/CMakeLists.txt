set(SRCS
    "main.cpp"
    "UartHandler.cpp"
    "driver/PCA9685.cpp"
    "sound/DualI2SReader.cpp"
    "sound/VAD.cpp"
    "sound/SrpSoundLocalizer.cpp"
    "sound/SoundManager.cpp"
    "motion_manager/MotionController.cpp"
    "motion_manager/MotionStorage.cpp"
    "motion_manager/ActionManager.cpp"
    "motion_manager/DecisionMaker.cpp"
    "web_server/WebServer.cpp"
)

set(INCLUDE_DIRS
    "./"
    "driver"
    "sound"
    "motion_manager"
    "web_server"
)

set(EMBED_TXTFILES
    "www/index.html"
)

idf_component_register(SRCS ${SRCS} INCLUDE_DIRS ${INCLUDE_DIRS} EMBED_TXTFILES ${EMBED_TXTFILES})

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
