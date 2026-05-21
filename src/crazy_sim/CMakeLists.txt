cmake_minimum_required(VERSION 3.10)
project(crazy_sim)


find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------------
# Install all data files (worlds, models, config, launch).
# These are pure data — no compilation needed.
# -----------------------------------------------------------------------
install(DIRECTORY
  worlds
  models
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()