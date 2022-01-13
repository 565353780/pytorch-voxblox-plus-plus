# inspired by Open3D/cmake/Open3DMakeCudaArchitectures.cmake
# detect_cuda_architectures(cuda_archs)
#
# Sets up CUDA architectures based on the following precedence rules
# and stores them into the <cuda_archs> variable.
#   1. User-defined architectures
#   2. Architectures detected on the current machine
#   3. CMake's default architectures
function(detect_cuda_architectures cuda_archs)
    unset(${cuda_archs})

    find_package(CUDA REQUIRED)

    if(CMAKE_CUDA_ARCHITECTURES)
        set(${cuda_archs} ${CMAKE_CUDA_ARCHITECTURES})
        message(STATUS "Building with user-provided architectures")
    else()
        file(WRITE
            "${CMAKE_CURRENT_BINARY_DIR}/cuda_architectures.c"
            "
            #include <stdio.h>
            #include <cuda_runtime_api.h>
            int main() {
                int n;
                if (cudaGetDeviceCount(&n) == cudaSuccess) {
                    for (int i = 0; i < n; ++i) {
                        int major, minor;
                        if (cudaDeviceGetAttribute(&major, cudaDevAttrComputeCapabilityMajor,
                                                i) == cudaSuccess &&
                            cudaDeviceGetAttribute(&minor, cudaDevAttrComputeCapabilityMinor,
                                                i) == cudaSuccess) {
                            if (i > 0) {
                                printf(\";\");
                            }
                            printf(\"%d%d-real\", major, minor);
                        }
                    }
                }
                return 0;
            }
            ")

        try_run(
            DETECTION_RETURN_VALUE DETECTION_COMPILED
            "${CMAKE_CURRENT_BINARY_DIR}"
            "${CMAKE_CURRENT_BINARY_DIR}/cuda_architectures.c"
            LINK_LIBRARIES ${CUDA_LIBRARIES}
            CMAKE_FLAGS "-DINCLUDE_DIRECTORIES=${CUDA_INCLUDE_DIRS}"
            RUN_OUTPUT_VARIABLE DETECTED_ARCHITECTURES)

        if(DETECTED_ARCHITECTURES)
          message(STATUS "Building with detected architectures ${DETECTED_ARCHITECTURES}")
            set(${cuda_archs} ${DETECTED_ARCHITECTURES})
        else()
            message(STATUS "Failed to detect architectures. Falling back to CMake's default architectures")
        endif()
    endif()

    set(${cuda_archs} ${${cuda_archs}} PARENT_SCOPE)

endfunction()
