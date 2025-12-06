# Architectural Decisions: Physical AI & Humanoid Robotics Textbook

This document records significant architectural decisions made during the development of the Physical AI & Humanoid Robotics textbook.

## ADR-001: Primary Programming Language
**Date:** 2025-12-05
**Status:** Accepted

**Context:**
The textbook needs to include working code examples that readers can execute and understand. We need to choose a programming language that balances accessibility for students with capability for robotics applications.

**Decision:**
Use Python 3.10+ as the primary language for code examples (80% of content).

**Rationale:**
- Most accessible to target audience (undergraduate students, hackathon participants)
- Rich ecosystem (numpy, scipy, matplotlib, opencv) for robotics
- Jupyter notebook support for interactive learning
- Cross-platform compatibility
- Industry adoption in robotics and AI

**Consequences:**
- ✅ Easy for beginners to follow and modify
- ✅ Quick prototyping and visualization
- ⚠️ Performance limitations (mitigated by using numpy for heavy computation)
- ⚠️ Not true real-time (acknowledged in text with alternatives mentioned)

**Alternatives Considered:**
- C++: Too complex for beginners, steeper learning curve
- MATLAB: Proprietary, limited accessibility for students
- Julia: Less mature ecosystem for robotics applications

## ADR-002: Simulation Platform
**Date:** 2025-12-05
**Status:** Accepted

**Context:**
The textbook needs to demonstrate concepts with executable examples. We must choose a simulation platform that readers can easily install and use.

**Decision:**
Use PyBullet as the primary simulator.

**Rationale:**
- pip-installable (no complex setup required)
- Works on Windows/Mac/Linux
- Python API (consistent with codebase)
- Free and open-source
- Adequate physics accuracy for educational purposes

**Consequences:**
- ✅ Low barrier to entry for students
- ✅ Fast iteration during development
- ⚠️ Less industry-standard than Gazebo (mitigated by providing Gazebo conversion notes)
- ⚠️ Limited to Python integration

**Alternatives Considered:**
- Gazebo: Linux-only, complex setup, ROS dependency
- Isaac Gym: GPU required, NVIDIA-specific
- MuJoCo: Recently open-sourced but still complex setup

## ADR-003: Document Format Chain
**Date:** 2025-12-05
**Status:** Accepted

**Context:**
The textbook needs to be distributed in a professional format while maintaining version control friendliness for development.

**Decision:**
Use Markdown source → Pandoc → PDF workflow.

**Rationale:**
- Markdown: Human-readable, version-control friendly
- Pandoc: Powerful conversion, supports LaTeX math
- PDF: Universal format for distribution and printing

**Consequences:**
- ✅ Easy to edit and collaborate on
- ✅ Git-friendly (line-based diffs)
- ✅ Automated builds via scripts
- ⚠️ Requires LaTeX installation for PDF generation (documented in setup)

**Alternatives Considered:**
- LaTeX directly: Steeper learning curve for non-technical contributors
- Word: Binary format, poor version control
- HTML/Ebook formats: No offline reading capability

## ADR-004: Citation Management
**Date:** 2025-12-05
**Status:** Accepted

**Context:**
The textbook must meet academic publication standards with proper citations in APA 7 format.

**Decision:**
Use BibTeX with APA 7 CSL style for citation management.

**Rationale:**
- BibTeX: Standard for academic publishing
- APA 7: Required citation style for academic work
- CSL: Pandoc-compatible styling
- Plain text format (version control friendly)

**Consequences:**
- ✅ Professional citations that meet academic standards
- ✅ Easy to add/modify references
- ✅ Automated formatting via build process

**Tools:**
- Zotero for managing references, export to BibTeX

## ADR-005: Figure Creation Tools
**Date:** 2025-12-05
**Status:** Accepted

**Context:**
The textbook requires 30+ professional figures with consistent styling and APA 7 compliant captions.

**Decision:**
Use mixed toolchain based on figure type with SVG export as primary format.

**Rationale:**
- Excalidraw: Fast, web-based, collaborative for diagrams
- Matplotlib: Scriptable, reproducible plots
- All export to SVG (vector format) for quality and scalability

**Tool Mapping:**
| Figure Type | Primary Tool | Fallback | Output Format |
|-------------|--------------|----------|---------------|
| Block diagrams | Excalidraw | draw.io | SVG |
| Mathematical plots | Matplotlib | Plotly | SVG/PNG 300 DPI |
| 3D visualizations | Matplotlib 3D | Blender | SVG/PNG 300 DPI |
| Circuit diagrams | draw.io | - | SVG |

**Consequences:**
- ✅ High-quality vector graphics that scale perfectly
- ✅ Source files editable for future modifications
- ⚠️ Multiple tools to learn (mitigated by providing tool guides)

## ADR-006: Code Quality Standards
**Date:** 2025-12-05
**Status:** Accepted

**Context:**
Code examples must be educational, correct, and follow professional software engineering practices.

**Decision:**
Implement strict code quality standards including type hints, docstrings, and testing.

**Rationale:**
- Educational value: Students learn professional practices
- Correctness: Reduces bugs and confusion
- Maintainability: Easier to update and modify

**Standards:**
- Type hints (PEP 484) for all functions
- Google-style docstrings with examples
- Input validation and error handling
- Unit tests for all algorithms
- Performance benchmarks documented

**Consequences:**
- ✅ Professional-quality code examples
- ✅ Educational value beyond just the algorithm
- ⚠️ More verbose than minimal examples (but more educational)

## ADR-007: Mathematical Depth
**Date:** 2025-12-05
**Status:** Accepted

**Context:**
The textbook must balance mathematical rigor with accessibility for students of varying mathematical backgrounds.

**Decision:**
Include full formulas with variable definitions and brief 2-3 step derivations, with full proofs cited.

**Rationale:**
- Academic rigor: Proper mathematical foundation
- Accessibility: Derivations show intuition without full proofs
- Credibility: All equations properly sourced

**Format:**
```
### Forward Kinematics Derivation

1. Transform from base to joint 1: T₁ = Rot_z(θ₁) ⋅ Trans_z(d₁) ⋅ Trans_x(a₁) ⋅ Rot_x(α₁)
2. Continue for each joint: T = T₁ ⋅ T₂ ⋅ ... ⋅ Tn
3. Final end-effector pose: Tₑₑ = T

Full derivation: Craig (2005, pp. 76-80)
```

**Consequences:**
- ✅ Mathematically rigorous but accessible
- ✅ Students understand the "why" not just the "what"
- ✅ Proper academic citations

## ADR-008: Real-time Implementation Notes
**Date:** 2025-12-05
**Status:** Accepted

**Context:**
The textbook covers control systems with real-time constraints, but Python cannot provide hard real-time guarantees.

**Decision:**
Provide simplified Python implementations with clear notes about real-time limitations and alternatives.

**Rationale:**
- Educational focus: Algorithms over real-time implementation
- Accessibility: Students can run examples without special hardware
- Practical value: Concepts transfer to real-time systems

**Approach:**
- Show algorithms in Python for clarity
- Note real-time limitations clearly
- Mention real-time alternatives (RT-Linux, Arduino, RTOS)

**Consequences:**
- ✅ Students can run and modify all examples
- ✅ Focus on algorithmic understanding
- ⚠️ Additional step needed for real-time deployment (clearly documented)

## ADR-009: Hardware Specifications
**Date:** 2025-12-05
**Status:** Accepted

**Context:**
The textbook should provide hardware examples but not lock readers into specific products that may become obsolete.

**Decision:**
Provide generic specifications with specific examples in appendices.

**Rationale:**
- Longevity: Textbook remains relevant as products change
- Flexibility: Readers can apply to their own hardware
- Educational: Focus on principles over products

**Approach:**
- Main text: Generic specs (e.g., "50Nm continuous torque motor")
- Appendix B: Specific product examples (e.g., "Example: Maxon EC 90")

**Consequences:**
- ✅ Textbook remains current longer
- ✅ Focus on principles rather than products
- ✅ Readers can adapt to their own hardware