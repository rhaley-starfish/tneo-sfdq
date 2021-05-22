# Copyright (C) 2007-2011 LuaDist.
# Created by Peter Drahoš, Peter Kapec
# Redistribution and use of this file is allowed according to the terms of the MIT license.
# For details see the COPYRIGHT file distributed with LuaDist.
# Please note that the package source code is licensed under its own license.
# Changed by Vinicius Jarina (viniciusjarina@gmail.com)
# Copyright (C) 2020 StarfishMedical
# Redistribution and use of this file is allowed according to the terms of the MIT license.

PROJECT ( lua_static C )
CMAKE_MINIMUM_REQUIRED ( VERSION 3.16 )
SET (LIB_LUA_VER "54" CACHE STRING "Suffix of version (for library name) e.g lua54")

#
# Include dir COPY
# If the source dir and the binary dir are not equals,
# copy the content of libzond/cinlude to $/include
# 
IF(NOT ${PROJECT_SOURCE_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	SET(TEMP_INCLUDE_LUA_DIR_FROM "${PROJECT_SOURCE_DIR}/include")
	SET(TEMP_INCLUDE_LUA_DIR_FROM "${PROJECT_SOURCE_DIR}/src")
	SET(TEMP_INCLUDE_LUA_DIR_TO   "${PROJECT_BINARY_DIR}")
	MESSAGE(STATUS "Copying include dirs from  ${TEMP_INCLUDE_LUA_DIR_FROM} to ${TEMP_INCLUDE_LUA_DIR_TO}")
	FILE ( COPY ${TEMP_INCLUDE_LUA_DIR_FROM} DESTINATION ${TEMP_INCLUDE_LUA_DIR_TO})
ENDIF()

# Sources and headers
INCLUDE_DIRECTORIES ( include )
SET (   SRC_LIB
		src/lauxlib.c
		src/liolib.c
		src/lopcodes.c
		src/lstate.c
		src/lobject.c
		src/lmathlib.c
		src/loadlib.c
		src/lvm.c
		src/lfunc.c
		src/lstrlib.c
		src/linit.c
		src/lstring.c
		src/lundump.c
		src/lctype.c
		src/ltable.c
		src/ldump.c
		src/loslib.c
		src/lgc.c
		src/lzio.c
		src/ldblib.c
		src/lutf8lib.c
		src/lmem.c
		src/lcorolib.c
		src/lcode.c
		src/ltablib.c
		src/lapi.c
		src/lbaselib.c
		src/ldebug.c
		src/lparser.c
		src/llex.c
		src/ltm.c
		src/ldo.c
		)


ADD_LIBRARY ( ${PROJECT_NAME} STATIC ${SRC_LIB} )
TARGET_LINK_LIBRARIES ( ${PROJECT_NAME} ${LIBS} )
SET_TARGET_PROPERTIES ( ${PROJECT_NAME} PROPERTIES OUTPUT_NAME lua_static CLEAN_DIRECT_OUTPUT 1 )

target_compile_options(${PROJECT_NAME} PUBLIC
		-mcpu=cortex-m4
		-mthumb
		-mfloat-abi=hard
		-mfpu=fpv4-sp-d16
		-fdata-sections
		-ffunction-sections
		-Wall
		-Os
		-g
		)

target_link_options(${PROJECT_NAME} PUBLIC
		-T../src/STM32F446RETx_FLASH.ld
#		-specs=nosys.specs
		-mcpu=cortex-m4
		-mthumb
		-mfpu=fpv4-sp-d16
		-mfloat-abi=hard
		-Wl,-Map=${PROJECT_NAME}.map,--cref
		-Wl,--gc-sections
		)

add_custom_command(TARGET ${PROJECT_NAME}
		POST_BUILD
		COMMAND ${CMAKE_SIZE_UTIL} lib${PROJECT_NAME}.a
		)

ENABLE_TESTING()

SET(TEST_PARAM -e _U=true)

SET(LUA_LIBRARIES ${PROJECT_NAME} PARENT_SCOPE)
SET(LUA_INCLUDE_DIR "src/" PARENT_SCOPE)

ADD_TEST(NAME test_all
         WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test 
		 COMMAND lua ${TEST_PARAM} ${PROJECT_SOURCE_DIR}/test/all.lua)
