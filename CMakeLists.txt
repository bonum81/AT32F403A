cmake_minimum_required(VERSION 3.15.3)

project(ARTERY VERSION 0.0.1 LANGUAGES CXX C ASM)

enable_language(C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

#set(ARCH_FLAGS "-mcpu=cortex-m3 -mthumb")
#set(OPT_FLAGS "-O2")
#set(N_OPT_FLAGS "-O0 -ggdb3")


# Add preprocessor definitions to the compilation of source files
add_compile_definitions(AT32F407AVGT7)
#add_compile_definitions(USE_JTAG_A)
add_compile_definitions(__START=main)
#add_compile_definitions(__STARTUP_CLEAR_BSS)


add_compile_options(
	#-march=armv7e-m
	#-mcpu=cortex-m4
	#-mthumb
	#-mfpu=fpv4-sp-d16
	#-mfloat-abi=hard
	-O0
	-ggdb3					# info for gdb
	-Wall
	-Wextra
	-Wshadow
	#-Wsign-conversion
	-ffunction-sections
	-fdata-sections
	-Wl,--gc-sections 		# delete not use code
	$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)


add_link_options(
	-T${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/device_support/startup/gcc/linker/AT32F403AxG_FLASH.ld
	-lc
	-lm
	-lnosys
	-lstdc++
	--specs=nano.specs
	--specs=nosys.specs
	-Wl,--gc-sections
	-Wl,--print-memory-usage
	-Wl,--undefined=uxTopUsedPriority
	-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)


#Add directory librares
add_subdirectory(libraries)

#Add main includes directory
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/MAIN/include/
)



#Add main src files
set(SRC
	${CMAKE_CURRENT_SOURCE_DIR}/MAIN/app.cpp
	)



add_executable(${PROJECT_NAME} ${SRC})
#add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} stdlibraries)
#target_link_libraries(${PROJECT_NAME} freertos)

#set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
#set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD_REQUIRED ON)


# function (createBinAndHexFiles TARGET_NAME)
    # add_custom_command(TARGET ${TARGET_NAME}
        # POST_BUILD
        # COMMAND ${CMAKE_SIZE_UTIL} ${TARGET_NAME}.out
        # COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET_NAME}.out "${TARGET_NAME}.bin"
        # COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET_NAME}.out "${TARGET_NAME}.hex"
        # COMMENT "post build steps for ${TARGET_NAME}"
    # )
# endfunction()

# createBinAndHexFiles(${PROJECT_NAME})

add_custom_target(hex
	ALL
	COMMAND ${CMAKE_OBJCOPY} ${PROJECT_NAME} ${PROJECT_NAME}.elf
	COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME} ${PROJECT_NAME}.hex
	COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME} ${PROJECT_NAME}.bin
	DEPENDS ${PROJECT_NAME}
	COMMENT "Generating Hex file ${PROJECT_NAME}.hex"
)
