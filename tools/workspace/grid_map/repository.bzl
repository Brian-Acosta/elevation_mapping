load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def grid_map_repository(grid_map_commit = "main",
                        grid_map_checksum = "82bbf39625d19af20cd57f33948b53eb52e8fc72a667f18cff837b2c50c4dd73"):
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
