

# list of files in this directory
file(GLOB SOURCES *.cpp glsl/*.cpp queries/*.cpp)
file(GLOB HEADER_FILES *.h glsl/*.h queries/*.h)

install(
    FILES ${HEADER_FILES}
    DESTINATION ${HEADER_INSTALL_PATH}/gl-types
)
