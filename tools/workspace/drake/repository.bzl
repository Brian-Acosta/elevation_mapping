load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def drake_repository(
        drake_commit = "v1.21.0",
        drake_checksum = "6571295843aff8e11620340739bf5eab7a25130f8f06667b2d3e6df85567509a"):
    maybe(
        http_archive,
        name = "drake",
        sha256 = drake_checksum,
        strip_prefix = "drake-{}".format(
            drake_commit.strip("v"),
        ),
        urls = [x.format(drake_commit) for x in [
            "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
        ]],
    )
