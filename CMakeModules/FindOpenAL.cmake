include(Utility)

if (APPLE)
    find_include_path(OPENAL NAMES OpenAL/al.h)
else()
    find_include_path(OPENAL NAMES AL/al.h)
endif()

find_library_path(OPENAL NAMES libopenal.a OpenAL al openal soft_oal OpenAL32)

# handle the QUIETLY and REQUIRED arguments and set XXX_FOUND to TRUE if all listed variables are TRUE
include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OpenAL  DEFAULT_MSG  OPENAL_LIBRARIES OPENAL_INCLUDE_DIRS)

