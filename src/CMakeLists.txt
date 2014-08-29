SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(.)

# ##############################################################################
# Find required libraries
# ##############################################################################

# Find Boost, at least ver. 1.41
FIND_PACKAGE(Boost 1.41.0 REQUIRED)
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# Find Boost
find_package( Boost COMPONENTS thread system program_options filesystem  REQUIRED ) #whatever libs you need
include_directories(SYSTEM ${Boost_INCLUDE_DIR})


# # OpenCV library
# FIND_PACKAGE( OpenCV REQUIRED )

# Find another DCLs our one depends on
# Use macro: DISCODE_FIND_DCL(<DCL_NAME>)

# ##############################################################################
# Add source directories
# ##############################################################################

# CvBlobs components
ADD_SUBDIRECTORY(Components)

# CvBlobs types
#ADD_SUBDIRECTORY(Types)

# Prepare config file to use from another DCLs
CONFIGURE_FILE(MongoDBConfig.cmake.in ${CMAKE_INSTALL_PREFIX}/MongoDBConfig.cmake @ONLY)
