armino_build_get_property(armino_soc ARMINO_SOC)
set(incs )
armino_component_register(INCLUDE_DIRS "${incs}")

armino_build_get_property(toolchain_dir TOOLCHAIN_DIR)
set(gcc_compiler ${toolchain_dir}/riscv32-elf-gcc)
set(toolchain_version_file ${CMAKE_CURRENT_SOURCE_DIR}/${armino_soc}/hash/toolchain_version.txt)
# bk_libs hash verify disabled
execute_process(COMMAND
				"${gcc_compiler}" --version
				OUTPUT_VARIABLE sdk_toolchain_version
				RESULT_VARIABLE result)
file(WRITE ${toolchain_version_file} "${sdk_toolchain_version}")

file(GLOB common_libs ${CMAKE_CURRENT_SOURCE_DIR}/common/*.a)
foreach(lib ${common_libs})
	get_filename_component(lib_name ${lib} NAME)
	add_prebuilt_library(${lib_name} "${CMAKE_CURRENT_SOURCE_DIR}/common/${lib_name}")
	target_link_libraries(${COMPONENT_LIB} INTERFACE ${lib_name})
endforeach()

file(GLOB libs ${CMAKE_CURRENT_SOURCE_DIR}/${armino_soc}/libs/*.a)

set (blacklist_elem
	libbluetooth_host_ble[^.]*\.a
	libbluetooth_controller[^.]*\.a
	libbluetooth_host_dm[^.]*\.a
	libbk_phy[^.]*\.a
	libcom_phy[^.]*\.a
	libate_phy[^.]*\.a
	libtjpg[^.]*\.a
	libscale[^.]*\.a
	libwifi[^.]*\.a
)

list (JOIN blacklist_elem | blacklist_elem_regax)


list(FILTER libs EXCLUDE REGEX "^.*(${blacklist_elem_regax})$")

foreach(lib ${libs})
	#message("final lib ${lib}")
endforeach()

foreach(lib ${libs})
 	get_filename_component(lib_name ${lib} NAME)
 	add_prebuilt_library(${lib_name} "${CMAKE_CURRENT_SOURCE_DIR}/${armino_soc}/libs/${lib_name}")
 	target_link_libraries(${COMPONENT_LIB} INTERFACE ${lib_name})
endforeach()

