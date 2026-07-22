# ── unit tests ──────────────────────────────────────────────────────────────
set(TEST_NAMES
  astar_test
  bfs_test
  route_test
  dstarlite_test
)

foreach(name IN LISTS TEST_NAMES)
  string(REPLACE "_test" "" base ${name})
  ament_add_gtest(${name} ${name}.cpp)
  target_link_libraries(${name}
    finenav_route_planner_lib
    ${base}_params
  )
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()
