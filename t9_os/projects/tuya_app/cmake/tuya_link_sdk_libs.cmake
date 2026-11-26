# Link TuyaOpen static libraries produced by top-level `make app` (libtuyaopen.a, lib<APP_NAME>.a)
# into the AP firmware only. CP does not need the full SDK archive.

if(NOT "${ARMINO_SOC}" MATCHES "_ap$")
    return()
endif()

if(DEFINED ENV{TUYA_PROJECT_DIR} AND NOT "$ENV{TUYA_PROJECT_DIR}" STREQUAL "")
    get_filename_component(_tuya_sdk_root "$ENV{TUYA_PROJECT_DIR}" ABSOLUTE)
else()
    get_filename_component(_tuya_sdk_root "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)
endif()

set(_lib_dir "")
if(EXISTS "${_tuya_sdk_root}/output/T9_tuyaos-ai/lib/libtuyaopen.a")
    set(_lib_dir "${_tuya_sdk_root}/output/T9_tuyaos-ai/lib")
endif()
if(_lib_dir STREQUAL "")
    file(GLOB _candidate_dirs "${_tuya_sdk_root}/output/*/lib")
    list(SORT _candidate_dirs)
    foreach(_d IN LISTS _candidate_dirs)
        if(EXISTS "${_d}/libtuyaopen.a")
            set(_lib_dir "${_d}")
            break()
        endif()
    endforeach()
endif()

set(_libtuyaopen "")
set(_app_lib "")
if(NOT _lib_dir STREQUAL "")
    if(EXISTS "${_lib_dir}/libtuyaopen.a")
        set(_libtuyaopen "${_lib_dir}/libtuyaopen.a")
    endif()
    if(DEFINED ENV{APP_NAME} AND NOT "$ENV{APP_NAME}" STREQUAL "")
        set(_app_name "$ENV{APP_NAME}")
        if(EXISTS "${_lib_dir}/lib${_app_name}.a")
            set(_app_lib "${_lib_dir}/lib${_app_name}.a")
        endif()
    endif()
endif()

if(NOT TARGET app.elf)
    return()
endif()

# Plain signature only: project.cmake already uses plain target_link_libraries(app.elf ...);
# CMake forbids mixing plain and keyword (PRIVATE) on the same target.
if(EXISTS "${_libtuyaopen}" AND EXISTS "${_app_lib}")
    target_link_libraries(app.elf
        "-Wl,--start-group"
        "${_libtuyaopen}"
        "${_app_lib}"
        "-Wl,--end-group"
    )
    message(STATUS "Tuya SDK: link group ${_libtuyaopen} + ${_app_lib}")
elseif(EXISTS "${_libtuyaopen}")
    target_link_libraries(app.elf "${_libtuyaopen}")
    message(STATUS "Tuya SDK: link ${_libtuyaopen}")
elseif(EXISTS "${_app_lib}")
    target_link_libraries(app.elf "${_app_lib}")
    message(STATUS "Tuya SDK: link ${_app_lib}")
endif()

if(NOT EXISTS "${_libtuyaopen}")
    message(WARNING "Tuya SDK: libtuyaopen.a not found under ${_tuya_sdk_root}/output/*/lib — run top-level `make app` (OS + app) before Armino AP link.")
endif()
if(NOT EXISTS "${_app_lib}" AND DEFINED ENV{APP_NAME} AND NOT "$ENV{APP_NAME}" STREQUAL "")
    message(WARNING "Tuya SDK: lib$ENV{APP_NAME}.a not found — set APP_NAME when building (e.g. make APP_NAME=t5_lowpower_pair_demo).")
endif()
