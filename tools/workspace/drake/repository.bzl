load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def drake_repository(
        drake_commit = "v1.22.0",
        drake_checksum = "78cf62c177c41f8415ade172c1e6eb270db619f07c4b043d5148e1f35be8da09"):
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
