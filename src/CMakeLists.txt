add_library(slamAR SHARED camera.cpp frame.cpp mappoint.cpp map.cpp config.cpp vo.cpp g2o_types.cpp)
target_link_libraries(slamAR ${THIRD_PARTY_LIBS})
