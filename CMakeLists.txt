cmake_minimum_required(VERSION 3.15)

project(triangles)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} main.cpp Vulkan/driver.cpp Intersection_Geometry/geometry.cpp Intersection_Geometry/Octotree.cpp Intersection_Geometry/Space.cpp Loader.cpp)

set(FB_TARGET bs_pre_vulkan3)
add_custom_target(${FB_TARGET}
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/tests ${CMAKE_CURRENT_BINARY_DIR}/tests
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Vulkan/shaders ${CMAKE_CURRENT_BINARY_DIR}/Vulkan/shaders

        COMMAND glslc ${CMAKE_CURRENT_BINARY_DIR}/Vulkan/shaders/shader.vert -o ${CMAKE_CURRENT_BINARY_DIR}/Vulkan/shaders/vert.spv
        COMMAND glslc ${CMAKE_CURRENT_BINARY_DIR}/Vulkan/shaders/shader.frag -o ${CMAKE_CURRENT_BINARY_DIR}/Vulkan/shaders/frag.spv
        )

find_package(Vulkan REQUIRED)
find_package(X11 REQUIRED)
find_package(glfw3 REQUIRED)

target_include_directories(${PROJECT_NAME} PRIVATE "$ENV{VULKAN_SDK}/include")
target_link_directories(${PROJECT_NAME} PRIVATE "$ENV{VULKAN_SDK}/lib")
target_link_directories(${PROJECT_NAME} PRIVATE "$ENV{VULKAN_SDK}/etc/explicit_layer.d")

# Link dependencies
target_link_libraries(${PROJECT_NAME} glfw)
target_link_libraries(${PROJECT_NAME} Vulkan::Vulkan)
target_link_libraries(${PROJECT_NAME} ${X11_LIBRARIES})

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/tests $<TARGET_FILE_DIR:${PROJECT_NAME}>/tests)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Vulkan/shaders $<TARGET_FILE_DIR:${PROJECT_NAME}>/Vulkan/shaders)

add_dependencies(${PROJECT_NAME} ${FB_TARGET})