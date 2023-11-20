load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def drake_repository(
        drake_commit = "v1.23.0",
        drake_checksum = "2e64bca9d5fe942170617d8109ec7ffe5df095d821743c9a619d38599229d03f"):
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
