git_repository(
    name = "bazel_rules",
    remote = "https://github.com/antonovvk/bazel_rules",
    commit = "36c56e5b96731d01693500f86dcb23ff9b405e34",
)

new_git_repository(
    name = "glog_repo",
    remote = "https://github.com/google/glog.git",
    commit = "b6a5e0524c28178985f0d228e9eaa43808dbec3c",
    build_file = "bzl/glog.BUILD"
)

bind(
    name = "glog",
    actual = "@glog_repo//:glog"
)

git_repository(
    name = "com_github_gflags_gflags",
    remote = "https://github.com/gflags/gflags.git",
    tag = "v2.2.1",
)

bind(
    name = "gflags",
    actual = "@com_github_gflags_gflags//:gflags",
)
