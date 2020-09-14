# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "darshanhegde/models",
        commit = "b7bf29dae324177dd72bdb71945726ab7e2a3d61",
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        local_repository_override="/home/darshanhegde/source/models",
        mirrors=mirrors
    )
