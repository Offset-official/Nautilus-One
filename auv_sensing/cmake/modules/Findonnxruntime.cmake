find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_c_api.h 
          PATHS /usr/local/onnxruntime-linux-x64-1.4.0/include/)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime 
             PATHS /usr/local/onnxruntime-linux-x64-1.4.0/lib/)

if (ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  set(onnxruntime_FOUND TRUE)
  set(onnxruntime_INCLUDE_DIRS ${ONNXRUNTIME_INCLUDE_DIR})
  set(onnxruntime_LIBRARIES ${ONNXRUNTIME_LIBRARY})
else()
  set(onnxruntime_FOUND FALSE)
endif()

mark_as_advanced(ONNXRUNTIME_INCLUDE_DIR ONNXRUNTIME_LIBRARY)
