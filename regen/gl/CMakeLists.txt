
# list of files in this directory
file(GLOB SOURCES *.cpp queries/*.cpp states/*.cpp)
file(GLOB HEADER_FILES *.h)
file(GLOB HEADER_FILES1 queries/*.h)
file(GLOB HEADER_FILES2 states/*.h)
file(GLOB SHADER_FILES *.glsl)

install(
    FILES ${HEADER_FILES}
    DESTINATION ${HEADER_INSTALL_PATH}/gl)
install(
    FILES ${HEADER_FILES1}
    DESTINATION ${HEADER_INSTALL_PATH}/gl/queries)
install(
    FILES ${HEADER_FILES2}
    DESTINATION ${HEADER_INSTALL_PATH}/gl/states)
install(
    FILES ${SHADER_FILES}
    DESTINATION ${SHADER_INSTALL_PATH}/gl)
