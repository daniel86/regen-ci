
file(GLOB SHADER_FILES *.glsl)

install(
    FILES ${SHADER_FILES}
    DESTINATION ${SHADER_INSTALL_PATH}/objects/gui)
