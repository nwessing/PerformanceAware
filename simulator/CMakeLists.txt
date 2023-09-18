file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/**.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/**.c")
file(GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/**.h")

add_executable(Simulator
  ${HEADER_FILES}
  ${SOURCE_FILES}
)

if (MSVC)
  target_compile_options(Simulator PRIVATE /W4 /WX)
endif()

target_include_directories(Simulator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(Simulator PRIVATE cxx_std_20)

target_compile_definitions(Simulator PRIVATE
  WIN32_LEAN_AND_MEAN
  NOMINMAX
  _CRT_SECURE_NO_WARNINGS
)

