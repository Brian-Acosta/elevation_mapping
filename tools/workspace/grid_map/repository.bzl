load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def grid_map_repository(
        grid_map_commit = "main",
        grid_map_checksum = "f4504ddce24569c36c1af2f35f60b6e1e631094443a6aa529a519df4b1d05189"):
    maybe(
        http_archive,
        name = "grid_map",
        sha256 = grid_map_checksum,
        strip_prefix = "grid_map_bazel-{}".format(
            grid_map_commit.strip("v"),
        ),
        urls = [x.format(grid_map_commit) for x in [
            "https://github.com/Brian-Acosta/grid_map_bazel/archive/{}.tar.gz",
        ]],
    )
