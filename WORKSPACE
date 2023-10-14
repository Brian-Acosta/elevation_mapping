workspace(name = "elevation_mapping")

load("//tools/workspace:deps.bzl", "add_elevation_mapping_dependencies")

add_elevation_mapping_dependencies()

load("@drake//tools/workspace:default.bzl", "add_default_workspace")

add_default_workspace()

load("@grid_map//tools/workspace:deps.bzl", "add_grid_map_dependencies")

add_grid_map_dependencies()

load("@rules_pcl//bzl:repositories.bzl", "pcl_repositories")

# exclude dependencies brought in by drake
pcl_repositories(
    excludes = [
        "gtest",
        "eigen",
        "libpng",
        "zlib",
    ],
)

load("@grid_map//tools/workspace/pcl:setup.bzl", "setup_pcl")

setup_pcl()
