
# list of all files in this directory
file(GLOB SOURCES *.cpp)
file(GLOB HEADER_FILES *.h)
file(GLOB SHADER_FILES *.glsl)

install(
    FILES ${HEADER_FILES}
    DESTINATION ${HEADER_INSTALL_PATH}/simulation)
install(
    FILES ${SHADER_FILES}
    DESTINATION ${SHADER_INSTALL_PATH}/simulation)
