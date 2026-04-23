cmake_minimum_required(VERSION 3.5)

project(CGAL-apps LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# make a bin folder to host all the executables
make_directory(${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

#####################################################################################################

##TCLAP
set (TCLAP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/tclap/include)
include_directories (${TCLAP_DIR})

#####################################################################################################

find_package (Boost)

if (NOT Boost_FOUND AND MSVC)
    if(NOT DEFINED ${CMAKE_TOOLCHAIN_FILE})
        #if(NOT DEFINED ENV{VCPKG_ROOT})
            #if(WIN32)
                set(VCPKG_ROOT $ENV{HOMEDRIVE}$ENV{HOMEPATH}/vcpkg_cpptruths)
            #else()
            #    set(VCPKG_ROOT $ENV{HOME}/.vcpkg_cpptruths)
            #endif()
        #else()
        #    set(VCPKG_ROOT $ENV{VCPKG_ROOT})
        #endif()

        if(NOT EXISTS ${VCPKG_ROOT})
            message("Cloning vcpkg in ${VCPKG_ROOT}")
            execute_process(COMMAND git clone https://github.com/Microsoft/vcpkg.git ${VCPKG_ROOT})
            # If a reproducible build is desired (and potentially old libraries are # ok), uncomment the
            # following line and pin the vcpkg repository to a specific githash.
            # execute_process(COMMAND git checkout 745a0aea597771a580d0b0f4886ea1e3a94dbca6 WORKING_DIRECTORY ${VCPKG_ROOT})
        else()
            # The following command has no effect if the vcpkg repository is in a detached head state.
            message("Auto-updating vcpkg in ${VCPKG_ROOT}")
            execute_process(COMMAND git pull WORKING_DIRECTORY ${VCPKG_ROOT})
        endif()

        if(NOT EXISTS ${VCPKG_ROOT}/README.md)
            message(FATAL_ERROR "***** FATAL ERROR: Could not clone vcpkg *****")
        endif()

        if(WIN32)
            set(BOOST_INCLUDEDIR ${VCPKG_ROOT}/installed/x86-windows/include)
            set(VCPKG_EXEC ${VCPKG_ROOT}/vcpkg.exe)
            set(VCPKG_BOOTSTRAP ${VCPKG_ROOT}/bootstrap-vcpkg.bat)
        else()
            set(VCPKG_EXEC ${VCPKG_ROOT}/vcpkg)
            set(VCPKG_BOOTSTRAP ${VCPKG_ROOT}/bootstrap-vcpkg.sh)
        endif()

        if(NOT EXISTS ${VCPKG_EXEC})
            message("Bootstrapping vcpkg in ${VCPKG_ROOT}")
            execute_process(COMMAND ${VCPKG_BOOTSTRAP} WORKING_DIRECTORY ${VCPKG_ROOT})
        endif()

        if(NOT EXISTS ${VCPKG_EXEC})
            message(FATAL_ERROR "***** FATAL ERROR: Could not bootstrap vcpkg *****")
        endif()

        set(CMAKE_TOOLCHAIN_FILE ${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake CACHE STRING "")

        #message(STATUS "***** Checking project third party dependencies in ${VCPKG_ROOT} *****")
        set(VCPKG_PLATFORM_TOOLSET v142)

        execute_process(
            COMMAND ${VCPKG_EXEC} install boost --triplet x64-windows-static --recurse
            WORKING_DIRECTORY ${VCPKG_ROOT})

        execute_process(
            COMMAND ${VCPKG_EXEC} install liblas --triplet x64-windows-static --recurse
            WORKING_DIRECTORY ${VCPKG_ROOT})

        set (Boost_INCLUDE_DIRS "${VCPKG_ROOT}/installed/x64-windows-static/include" )
        set (Boost_INCLUDE_DIR "${Boost_INCLUDE_DIRS}" )


    endif()
endif()

include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen3 QUIET NO_MODULE)

set(EIGEN_EXTERNAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen)
set(EIGEN_EXTERNAL_INCLUDE_DIR "")
if (EXISTS ${EIGEN_EXTERNAL_DIR}/Eigen/Dense)
    set(EIGEN_EXTERNAL_INCLUDE_DIR ${EIGEN_EXTERNAL_DIR})
elseif (EXISTS ${EIGEN_EXTERNAL_DIR}/eigen/Eigen/Dense)
    set(EIGEN_EXTERNAL_INCLUDE_DIR ${EIGEN_EXTERNAL_DIR}/eigen)
endif()

#####################################################################################################

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/cgal/include)
find_package (CGAL REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/external/cgal NO_DEFAULT_PATH)

if (CGAL_VERSION VERSION_LESS "5.5")
  message(FATAL_ERROR "CGAL version ${CGAL_VERSION} found, but >= 5.5 is required.")
endif()

#message(${CGAL_INCLUDE_DIRS})
include_directories(${CGAL_INCLUDE_DIRS})

#####################################################################################################
##liblas

set(libLAS_INCLUDE_DIRS "")
set(libLAS_LIBRARY "")
set(LIBLAS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/LAStools)
set(LIBLAS_BUILD ${LIBLAS_DIR}/build-${CMAKE_SYSTEM_NAME})
set(LIBLAS_INSTALL ${LIBLAS_DIR}/install-${CMAKE_SYSTEM_NAME})

if (EXISTS ${LIBLAS_DIR}/CMakeLists.txt)

    set (LIBLAS_LIB ${LIBLAS_INSTALL}/lib/LASlib/libLASlib.a)

    file(MAKE_DIRECTORY ${LIBLAS_BUILD})
    file(MAKE_DIRECTORY ${LIBLAS_INSTALL})

    if (APPLE)
        set (LIBLAS_LIB ${LIBLAS_INSTALL}/lib/liblas.dylib)
    elseif (MSVC)
        set (LIBLAS_LIB ${LIBLAS_DIR}/LASlib/lib/Release/LASlib.lib)
    endif()

    add_custom_target(libLAS-build
        COMMAND ${CMAKE_COMMAND} -S ${LIBLAS_DIR} -B ${LIBLAS_BUILD} -DCMAKE_INSTALL_PREFIX=${LIBLAS_INSTALL} -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
        COMMAND ${CMAKE_COMMAND} --build ${LIBLAS_BUILD} --config Release
        BYPRODUCTS ${LIBLAS_LIB}
    )

    set (libLAS_INCLUDE_DIRS
        ${LIBLAS_DIR}/LASlib/inc
        ${LIBLAS_DIR}/LASzip/src
        ${LIBLAS_DIR}/LASzip/include/laszip
    )
    set (libLAS_LIBRARY ${LIBLAS_LIB} )

elseif (MSVC)
    find_path(libLAS_INCLUDE_DIR
        NAMES lasreader_las.hpp
    )
    if (libLAS_INCLUDE_DIR)
        set(libLAS_INCLUDE_DIRS ${libLAS_INCLUDE_DIR})
    endif()
    find_library(libLAS_LIBRARY
        NAMES LASlib laslib
    )
endif()

include_directories (${libLAS_INCLUDE_DIRS})

#####################################################################################################

if (EIGEN_EXTERNAL_INCLUDE_DIR)
    include_directories(${EIGEN_EXTERNAL_INCLUDE_DIR})
elseif (TARGET Eigen3::Eigen)
    get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    include_directories(${EIGEN3_INCLUDE_DIRS})
endif()

#####################################################################################################

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#####################################################################################################

add_executable(${PROJECT_NAME}-shape-detection-ransac src/main-ransac.cpp)
target_compile_features(${PROJECT_NAME}-shape-detection-ransac PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}-shape-detection-ransac PUBLIC CGAL::CGAL)
if (libLAS_INCLUDE_DIRS AND (NOT MSVC OR libLAS_LIBRARY))
    target_include_directories(${PROJECT_NAME}-shape-detection-ransac PRIVATE ${libLAS_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}-shape-detection-ransac PUBLIC ${libLAS_LIBRARY})
    if (TARGET libLAS-build)
        add_dependencies(${PROJECT_NAME}-shape-detection-ransac libLAS-build)
    endif()
endif()

if (EIGEN_EXTERNAL_INCLUDE_DIR OR TARGET Eigen3::Eigen)
    add_executable(${PROJECT_NAME}-shape-detection-reg-growing src/main-fit.cpp)
    target_compile_features(${PROJECT_NAME}-shape-detection-reg-growing PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}-shape-detection-reg-growing PUBLIC CGAL::CGAL)
    if (TARGET Eigen3::Eigen AND NOT EIGEN_EXTERNAL_INCLUDE_DIR)
        target_link_libraries(${PROJECT_NAME}-shape-detection-reg-growing PUBLIC Eigen3::Eigen)
    endif()

    add_executable(${PROJECT_NAME}-shape-detection-reg-growing-cyl src/main-fit-cylinder.cpp)
    target_compile_features(${PROJECT_NAME}-shape-detection-reg-growing-cyl PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}-shape-detection-reg-growing-cyl PUBLIC CGAL::CGAL)
    if (TARGET Eigen3::Eigen AND NOT EIGEN_EXTERNAL_INCLUDE_DIR)
        target_link_libraries(${PROJECT_NAME}-shape-detection-reg-growing-cyl PUBLIC Eigen3::Eigen)
    endif()
else()
    message(WARNING "Skipping ${PROJECT_NAME}-shape-detection-reg-growing: Eigen headers were not found.")
    message(WARNING "Skipping ${PROJECT_NAME}-shape-detection-reg-growing-cyl: Eigen headers were not found.")
endif()

if (libLAS_INCLUDE_DIRS AND (NOT MSVC OR libLAS_LIBRARY))
    add_executable(${PROJECT_NAME}-scaline-normals src/main-scanline-normals.cpp)
    target_compile_features(${PROJECT_NAME}-scaline-normals PUBLIC cxx_std_17)
    target_include_directories(${PROJECT_NAME}-scaline-normals PRIVATE ${libLAS_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}-scaline-normals PUBLIC ${libLAS_LIBRARY})
    if (TARGET libLAS-build)
        add_dependencies(${PROJECT_NAME}-scaline-normals libLAS-build)
    endif()
else()
    message(WARNING "Skipping ${PROJECT_NAME}-scaline-normals: LASlib headers/library were not found.")
endif()
