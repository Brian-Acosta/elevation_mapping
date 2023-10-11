load("@elevation_mapping//tools/workspace/drake:repository.bzl", "drake_repository")
load("@elevation_mapping//tools/workspace/grid_map:repository.bzl", "grid_map_repository")

def add_elevation_mapping_dependencies(excludes = []):
    if "drake" not in excludes:
        drake_repository()
    if "grid_map" not in excludes:
        grid_map_repository()