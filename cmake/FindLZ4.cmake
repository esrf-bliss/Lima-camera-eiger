find_path(LZ4_INCLUDE_DIRS NAMES lz4.h)
find_library(LZ4_LIBRARIES NAMES lz4)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LZ4
  DEFAULT_MSG
  LZ4_LIBRARIES
  LZ4_INCLUDE_DIRS)

mark_as_advanced(LZ4_INCLUDE_DIRS LZ4_LIBRARIES)
