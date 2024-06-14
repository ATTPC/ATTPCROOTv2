# How to Contribute to ATTPCROOT

> [!NOTE]
> These guidelines should be applicable to most contributions. At the same time, these are not 'one-size-fits-all' rules,
> and there might be cases where diverging from these guidelines is warranted. If you want to do something that doesn't fit in these guidelines, just ask!

## Bug reports, questions, feature requests, etc.
Create a [new issue](https://github.com/ATTPC/ATTPCROOTv2/issues/new/choose) on our GitHub.


## Your Code Contribution

The source code for ATTPCROOT is kept in [GitHub](https://github.com/ATTPC/ATTPCROOTv2).
Changes go through pull requests ("PRs").
The primary branch for development is `develop`.

> [!IMPORTANT]
> We require PRs to cleanly apply to develop without a merge commit, i.e. through "fast-forward".
> Please follow the [coding conventions](https://github.com/ATTPC/ATTPCROOTv2/wiki/Coding-conventions), as this is a simple item for
> reviewers to otherwise get stuck on.
> To make your (and our own) life easier, we provide a
> [`clang-format` configuration file](https://github.com/ATTPC/ATTPCROOTv2/blob/develop/.clang-format).


## Your Commit

Each commit is a self-contained, _atomic_ change. This means that:
1. **Each commit should be able to successfully build ATTPCROOT.**
Doing so makes traveling through the git history, for example during a `git bisect` much easier.
2. **Each commit does not contain more than one independent change.**
We define "one change" to mean a single shift in behavior or added feature. That could be many lines of code or entirely new classes. Once again, this makes review much easier as we can revert individual changes that cannot be merged yet (for whatever reason) more easily. If many changes are made in a single commit it gets messy to tease things out if something is broken.

> [!TIP]
> During a code review, it may be useful to make smaller commits to track intermediate changes, and rebase after the PR
> is approved to ensure the above points are met and to reduce clutter.

### Your Commit Message

The commit summary should not exceed 72 characters, be meaningful (i.e., it
describes the change) and should be written in the
[present imperative mood](https://git.kernel.org/pub/scm/git/git.git/tree/Documentation/SubmittingPatches?id=HEAD#n239)
(e.g. `Add this awesome feature` instead of `Adds this awesome feature` or `Added this awesome feature`).

The commit message that follow the summary can be used to provide more context to the change.
It should describe the **why**, rather than the **what** and **how** (we can gather this from the commit summary and the
change diff, respectively).
The commit message should be wrapped at 72 characters.


## Your Pull Request

> [!NOTE]
> For the mechanics on how to create pull requests, please visit
> [this page](https://root.cern/for_developers/creating_pr).

The title of your PR follows the same principle as the commit summary. If your PR only involves one commit, you can
reuse this summary. 

The PR description describes (and in case of multiple commits, summarizes) the change in more detail.
Again, try to describe the **why** (and in this case, to a lesser extent the **what**), rather than the **how**.

If your PR is related to an open [issue](https://github.com/ATTPC/ATTPCROOTv2/issues), make sure to link it.
This will be done automatically if you add
[closing keywords](https://docs.github.com/en/issues/tracking-your-work-with-issues/linking-a-pull-request-to-an-issue)
to the PR description.

Once a PR is created, one of the maintainers will review it (hopefully quickly). Please ping people :wave: should you not get timely feedback, for instance with `@ATTPC/attpcroot-managers  ping!`

## Tests

As you contribute code, this code will likely fix an issue or add a feature. 
Whatever it is: this requires you to add a new test, or to extend an existing test. Depending on the size and complexity
of this test, it exists either in the `test/` subdirectory of each part of ATTPCROOT (see for instance
[`AtData/test`](https://github.com/ATTPC/ATTPCROOTv2/tree/develop/AtData/test)), or in
[`macro/tests`](https://github.com/ATTPC/ATTPCROOTv2/tree/develop/macro/tests). 
We are just expanding out our coverage of tests, so you may need to add the `test/` subdirectory to whatever part of the code you are working in. 

Tests in `test/` subdirectories are unit tests, mostly based on
[Google Test](https://github.com/google/googletest) and easily extended. These unit tests should define the expected behavior of your code. They should not depend on any external resource (access to a file on the HD or network, etc.) Unit tests are only helpful if they are *fast*.

Tests in
[macro/tests](https://github.com/ATTPC/ATTPCROOTv2/tree/develop/macro/tests) are more involved (e.g., tests with
data files) and are run as interpreted macros rather than compiled unit tests.

## Continuous Integration

To prevent bad surprises and make a better first impression, we
strongly encourage new developers to [run the tests](https://github.com/ATTPC/ATTPCROOTv2/wiki/Running-tests)
_before_ submitting a pull request.

We have some automated (and not automated :cry:) CI tests that are used for pull requests:
- *Formatting check*: `clang-format` automatically checks that a PR
    [follows](https://github.com/ATTPC/ATTPCROOTv2/blob/develop/.clang-format) ATTPCROOT's
    [coding conventions](https://github.com/ATTPC/ATTPCROOTv2/wiki/Coding-conventions).
    If coding violations are found, it provides you with a `patch` output that you likely want to apply to your PR.
- *Simple Static Analysis*: PRs are analyzed using [`clang-tidy`](https://clang.llvm.org/extra/clang-tidy/) and [`iwyu`](https://include-what-you-use.org/).
- *Build and test*: Ensures all unit tests pass and the code builds.

Typically, PRs must pass all these tests; we will ask you to fix any issues that may arise.
Some tests are run only outside the PR testing system:
we might come back to you with additional reports after your contribution was merged.
