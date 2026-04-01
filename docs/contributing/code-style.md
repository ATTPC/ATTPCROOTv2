# Code Style

This page covers formatting and static-analysis tools only. Contributor-critical ROOT and persistence rules live in [guide.md](guide.md).

## Formatting

The project uses `clang-format-17` with the repo-root `.clang-format`.

Format one file:

```bash
clang-format-17 -i path/to/file.cxx
```

Format changed files:

```bash
scripts/formatAll.sh
```

## Static Analysis

The repo also carries `.clang-tidy` configuration and CI checks for:

- `clang-format`
- `clang-tidy`
- `iwyu`

Run clang-tidy manually on a file:

```bash
clang-tidy path/to/file.cxx -- -I...
```

For framework-specific contributor rules, use [guide.md](guide.md).
