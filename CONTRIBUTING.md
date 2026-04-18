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
4. Run the URLab automation suite. Two options:
   - Session Frontend in the Unreal Editor, or
   - Command line:
     ```
     UnrealEditor-Cmd.exe YourProject.uproject -ExecCmds="Automation RunTests URLab;Quit" -Unattended -NullRHI -Log
     ```
     The command-line form gives you output you can paste straight into the PR template.
5. Test your changes in PIE (Play in Editor).

## Submitting a Pull Request

1. Push to your fork and open a PR against `main`.
2. Fill in the [PR template](.github/pull_request_template.md). It asks for a summary, motivation, linked issue, **proof that the project builds**, and **proof that the `URLab.*` automation suite passes** on your machine — paste the tail of each command's output into the provided blocks.
3. We spot-check every PR in the editor before merging, but because Unreal Engine projects can't use standard CI the pasted build + test output is the primary gate. A PR without them will bounce back for that evidence.

## Code Style

- Keep comments concise. The code should explain itself. Use comments only for non-obvious decisions.
- No TODO/FIXME comments — open an issue instead.
- New MJCF attributes should follow the pattern in `/add-mjcf-attribute` (header property, ImportFromXml, ExportTo, test).
- New component types should follow the existing pattern: subclass `UMjComponent`, implement `RegisterToSpec`, `ExportTo`, `ImportFromXml`, `Bind`.

## Filing Issues

Use the [issue templates](.github/ISSUE_TEMPLATE) to file a bug or a feature request. They collect the environment, reproduction steps, and logs we need upfront — filling them in properly is the fastest path to a fix.

## License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.
