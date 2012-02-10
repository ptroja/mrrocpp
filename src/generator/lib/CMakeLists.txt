
add_library(ecp_smooth_base
	velocity_profile_calculator/bang_bang_profile.cc
	velocity_profile_calculator/constant_velocity_profile.cc
    velocity_profile_calculator/spline_profile.cc
	trajectory_interpolator/constant_velocity_interpolator.cc
	trajectory_interpolator/bang_bang_interpolator.cc
    trajectory_interpolator/spline_interpolator.cc
)

target_link_libraries(ecp_smooth_base)

install(TARGETS ecp_smooth_base DESTINATION lib)



