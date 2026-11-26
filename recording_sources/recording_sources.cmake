# Recording kinds (RecordingSource + HDMAPPING_REGISTER_RECORDING_SOURCE) compiled into the apps that load recordings.
# Each kind registers itself from its own .cpp, so these files must be compiled into the executable, not a static
# library. A new kind is added to RECORDING_SOURCE_FILES and needs no change in the apps.

# The registry and the Mandeye LAZ/CSV kind, which load_data(file names) uses directly.
set(RECORDING_SOURCE_CORE_FILES
  ${REPOSITORY_DIRECTORY}/recording_sources/recording_source.h
  ${REPOSITORY_DIRECTORY}/recording_sources/recording_source_factory.h
  ${REPOSITORY_DIRECTORY}/recording_sources/recording_source_factory.cpp
  ${REPOSITORY_DIRECTORY}/recording_sources/laz_csv_source.h
  ${REPOSITORY_DIRECTORY}/recording_sources/laz_csv_source.cpp
)

# Every kind. ROS1 bag / MCAP: the rosbags readers; McapWriter.cpp holds the single MCAP_IMPLEMENTATION that
# McapReader.cpp relies on.
set(RECORDING_SOURCE_FILES
  ${RECORDING_SOURCE_CORE_FILES}
  ${REPOSITORY_DIRECTORY}/recording_sources/bag_source.h ${REPOSITORY_DIRECTORY}/recording_sources/bag_source.cpp
  ${REPOSITORY_DIRECTORY}/recording_sources/bag_source_ui.h ${REPOSITORY_DIRECTORY}/recording_sources/bag_source_ui.cpp
  ${REPOSITORY_DIRECTORY}/rosbags/McapReader.cpp
  ${REPOSITORY_DIRECTORY}/rosbags/McapWriter.cpp
  ${REPOSITORY_DIRECTORY}/rosbags/Ros1Bag.cpp
)
