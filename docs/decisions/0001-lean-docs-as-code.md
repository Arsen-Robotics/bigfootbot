# ADR 0001: Lean "Docs-as-Code" Strategy

## Status
Accepted

## Context
The BigfootBot project is a complex, modular robotics system involving multiple hardware platforms (Jetson, RPi, Arduino). We need a documentation structure that is:
- Easy to maintain.
- Version-controlled alongside the code.
- "Single Source of Truth" (SSoT).
- Accessible within the IDE (Cursor) for developers and AI.

## Decision
We will adopt a **Lean "Docs-as-Code"** strategy using pure Markdown (`.md`) files instead of complex documentation generators (like Sphinx or MkDocs).

### Key Principles:
1.  **README-First**: Every functional unit (package, infrastructure stack, etc.) must have its own `README.md`.
2.  **Universal Naming**: Use `README.md` as the entry point for all directories to ensure automatic rendering on platforms like GitHub.
3.  **Diagrams-as-Code**: Use **Mermaid.js** syntax directly in Markdown files for architectural diagrams.
4.  **Decentralized Content**: High-level architecture lives in `/docs`, but package-specific technical "contracts" (topics, params, setup) live within the package folder.

## Consequences
- **Pros**:
    - Zero build steps required to read documentation.
    - AI (Cursor) can index and reason about the documentation much more effectively.
    - No maintenance overhead for documentation websites or themes.
- **Cons**:
    - No global search across a single generated website (must rely on IDE/GitHub search).
    - Limited advanced formatting (like cross-references between Python classes).
