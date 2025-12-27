
# list of files in this directory
file(GLOB SOURCES *.cpp)
file(GLOB HEADER_FILES *.h)
file(GLOB SHADER_FILES *.glsl)

install(
    FILES ${HEADER_FILES}
    DESTINATION ${HEADER_INSTALL_PATH}/objects)
install(
    FILES ${SHADER_FILES}
    DESTINATION ${SHADER_INSTALL_PATH}/objects)

foreach(DIR primitives lod terrain sky text particles gui)
    add_subdirectory(${DIR})
    get_directory_property(BUFFER DIRECTORY ${DIR} DEFINITION SOURCES)
    list(APPEND SOURCES ${BUFFER})
endforeach(DIR)
