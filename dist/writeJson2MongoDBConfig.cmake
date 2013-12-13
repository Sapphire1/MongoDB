# This file is used when other components needs to use something provided by this DCL. 
# Provide any include paths and lib directories. Use /home/lzmuda/DCL/writeJson2MongoDB/dist
# to point to 'dist' directory of current DCL, it'll be substituted during installation. 

# directory containing header files
SET(writeJson2MongoDB_INCLUDE_DIR /home/lzmuda/DCL/writeJson2MongoDB/dist/include)
INCLUDE_DIRECTORIES(${writeJson2MongoDB_INCLUDE_DIR})

# directory containing libraries
SET(writeJson2MongoDB_LIB_DIR /home/lzmuda/DCL/writeJson2MongoDB/dist/lib)
LINK_DIRECTORIES(${writeJson2MongoDB_LIB_DIR})

# list of libraries to link against when using features of writeJson2MongoDB
# add all additional libraries built by this dcl (NOT components)
# SET(writeJson2MongoDB_LIBS lib_1 lib_2)
# SET(ADDITIONAL_LIB_DIRS /home/lzmuda/DCL/writeJson2MongoDB/dist/lib ${ADDITIONAL_LIB_DIRS})
