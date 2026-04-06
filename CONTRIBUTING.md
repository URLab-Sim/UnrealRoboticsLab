# Contributing to URLab

Thank you for your interest in contributing. This document covers the process for submitting changes.

## Before You Start

- Check existing [issues](../../issues) to avoid duplicate work.
- For large changes (new features, architectural refactors), open an issue first to discuss the approach.

## Development Setup

1. Clone the repo into your UE project's `Plugins/` folder.
2. Build third-party libraries: `cd third_party && .\build_all.ps1`
3. Regenerate project files and build.
4. Verify the plugin loads in the Unreal Editor.

## Making Changes

1. Fork the repository and create a branch from `main`.
2. Make your changes. Follow existing code style:
   - UPROPERTYs use PascalCase (`Options`, `SimSpeedPercent`)
   - Non-UPROPERTY private members use `m_` prefix (`m_model`, `m_data`)
   - All `.h` and `.cpp` files must have the Apache 2.0 license header
3. Build and verify compilation succeeds.
4. Run the automation tests via the Session Frontend in the Unreal Editor, or use the `/test` command if you have Claude Code.
5. Test your changes in PIE (Play in Editor).

## Submitting a Pull Request

1. Push to your fork and open a PR against `main`.
2. In the PR description, include:
   - What the change does and why
   - How you tested it (which robots/XMLs, what scenarios)
   - Any breaking changes
3. Maintainers will manually build and test before merging, since Unreal Engine projects cannot use standard CI.

## Code Style

- Keep comments concise. The code should explain itself. Use comments only for non-obvious decisions.
- No TODO/FIXME comments — open an issue instead.
- New MJCF attributes should follow the pattern in `/add-mjcf-attribute` (header property, ImportFromXml, ExportTo, test).
- New component types should follow the existing pattern: subclass `UMjComponent`, implement `RegisterToSpec`, `ExportTo`, `ImportFromXml`, `Bind`.

## Reporting Issues

- Include the Unreal Engine version, OS, and the MuJoCo XML that reproduces the issue.
- Paste the relevant output log lines (`LogURLab`, `LogURLabImport`, `LogURLabNet`).
- If it's a compilation error, include the full error message.

## License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.
