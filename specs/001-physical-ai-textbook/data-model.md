# Data Model: Physical AI & Humanoid Robotics Textbook

**Date**: 2025-12-05
**Feature**: 001-physical-ai-textbook
**Status**: Complete

## Entity Models

### Chapter
**Description**: Organized content unit covering a specific topic in humanoid robotics

**Attributes**:
- `id`: String (chapter identifier, e.g., "ch01_introduction")
- `title`: String (chapter title)
- `number`: Integer (chapter number in sequence)
- `pages`: Integer (target page count)
- `figures_count`: Integer (number of figures in chapter)
- `code_examples_count`: Integer (number of code examples)
- `learning_objectives`: List[String] (measurable learning outcomes)
- `prerequisites`: List[String] (required background knowledge)
- `references`: List[Reference] (citations used in chapter)
- `practice_problems`: List[PracticeProblem] (exercises for students)

**Validation Rules**:
- `id` must follow pattern: ch[0-9]{2}_[a-z_]+
- `number` must be between 1-12
- `pages` must be between 6-14 (±20% from target)
- `learning_objectives` must have 3-5 items
- `practice_problems` must have 3-5 conceptual + 2-3 coding exercises

### Code Example
**Description**: Self-contained Python implementation demonstrating a specific algorithm or concept

**Attributes**:
- `id`: String (unique identifier within chapter)
- `title`: String (brief description of what it demonstrates)
- `language`: String (programming language, default: "Python")
- `version`: String (language version, e.g., "3.10+")
- `dependencies`: List[String] (required Python packages)
- `source_code`: String (the actual code implementation)
- `docstring`: String (Google-style documentation)
- `type_hints`: Boolean (whether type hints are included)
- `unit_tests`: List[String] (test cases to validate functionality)
- `performance`: PerformanceSpec (execution time and complexity data)

**Validation Rules**:
- `source_code` must execute without errors in clean environment
- `docstring` must follow Google style format
- `type_hints` must be present for all functions
- `unit_tests` must verify correctness with numerical validation

### PerformanceSpec
**Description**: Performance characteristics for code examples and algorithms

**Attributes**:
- `execution_time`: String (e.g., "<1ms for typical input")
- `time_complexity`: String (Big O notation, e.g., "O(n)")
- `space_complexity`: String (Big O notation, e.g., "O(1)")
- `hardware_reference`: String (reference hardware for benchmarks)
- `test_conditions`: String (conditions under which performance was measured)

**Validation Rules**:
- `execution_time` must be measurable and realistic
- `time_complexity` must match actual implementation
- `hardware_reference` must specify standard reference hardware

### Figure
**Description**: Professional diagram, plot, or schematic supporting textual explanations

**Attributes**:
- `id`: String (unique identifier within chapter)
- `title`: String (brief descriptive title)
- `type`: FigureType (block_diagram, mathematical_plot, hardware_schematic, photo_render)
- `format`: String (svg, png, pdf)
- `resolution`: Integer (DPI for raster images, 300 minimum)
- `source_file`: String (path to source/editable file)
- `caption`: String (APA 7 compliant caption)
- `alt_text`: String (accessibility description)
- `license`: String (usage rights information)

**Validation Rules**:
- `resolution` must be ≥300 DPI for raster images
- `caption` must follow APA 7 format
- `format` must be svg for vector images when possible

### FigureType
**Description**: Enumeration of figure types used in the textbook

**Values**:
- `block_diagram`: System architecture, control loops
- `mathematical_plot`: Graphs, phase portraits, trajectory plots
- `hardware_schematic`: Sensor wiring, kinematic chains, circuit diagrams
- `photo_render`: Robots, components, simulation screenshots

### Formula
**Description**: Mathematical expression with variable definitions and citations

**Attributes**:
- `id`: String (unique identifier within chapter)
- `expression`: String (LaTeX formatted equation)
- `variables`: List[VariableDefinition] (variable names with units and descriptions)
- `derivation`: String (brief 2-3 step derivation or citation to full derivation)
- `citation`: Reference (authoritative source for the formula)
- `validation_example`: String (numerical example validating the formula)
- `applicable_context`: String (when to use this formula)

**Validation Rules**:
- All variables in `expression` must be defined in `variables`
- `expression` must be dimensionally consistent
- `derivation` must be mathematically correct
- `citation` must point to authoritative source

### VariableDefinition
**Description**: Definition of a variable used in a formula

**Attributes**:
- `symbol`: String (mathematical symbol, e.g., "θ", "τ", "q")
- `name`: String (full name, e.g., "joint angle", "torque", "position vector")
- `units`: String (physical units, e.g., "rad", "Nm", "m")
- `description`: String (brief explanation of what the variable represents)
- `constraints`: String (valid range or special conditions, if applicable)

**Validation Rules**:
- `symbol` must match usage in formula
- `units` must be consistent with physical quantity
- `constraints` must be mathematically valid

### PracticeProblem
**Description**: Exercise testing understanding of chapter concepts

**Attributes**:
- `id`: String (unique identifier within chapter)
- `type`: ProblemType (conceptual, coding, analytical, design)
- `difficulty`: DifficultyLevel (easy, medium, hard)
- `description`: String (problem statement)
- `requirements`: List[String] (what the solution must include)
- `expected_outcome`: String (what constitutes a correct solution)
- `hints`: List[String] (optional guidance for students)
- `solution`: String (for solution manual, if applicable)

**Validation Rules**:
- `requirements` must be specific and measurable
- `expected_outcome` must be achievable with chapter knowledge
- `difficulty` must match actual problem complexity

### ProblemType
**Description**: Enumeration of practice problem types

**Values**:
- `conceptual`: Understanding of concepts, theory, principles
- `coding`: Implementation of algorithms or code modifications
- `analytical`: Mathematical problem solving using formulas
- `design`: System design or architecture decisions

### DifficultyLevel
**Description**: Difficulty rating for practice problems

**Values**:
- `easy`: Can be solved with direct application of concepts
- `medium`: Requires combination of multiple concepts
- `hard`: Requires deep understanding and problem-solving skills

### Reference
**Description**: Academic or technical source cited in the textbook

**Attributes**:
- `id`: String (unique identifier)
- `type`: ReferenceType (academic_paper, textbook, technical_report, standard, website)
- `author`: String (author name or organization)
- `year`: Integer (publication year)
- `title`: String (publication title)
- `journal`: String (journal or conference name, if applicable)
- `volume`: String (volume and issue number, if applicable)
- `pages`: String (page range, if applicable)
- `doi`: String (Digital Object Identifier, if available)
- `url`: String (URL for online resources)
- `accessed_date`: String (date accessed for online resources)

**Validation Rules**:
- Must follow APA 7 citation format
- `type` must match actual source type
- Required fields must be present based on `type`

### ReferenceType
**Description**: Enumeration of reference types

**Values**:
- `academic_paper`: Peer-reviewed journal or conference paper
- `textbook`: Published textbook
- `technical_report`: Technical report or white paper
- `standard`: Technical standard (ISO, IEEE, etc.)
- `website`: Online resource or documentation

## Relationships

### Chapter contains multiple
- Code Examples (1 to many)
- Figures (1 to many)
- Practice Problems (1 to many)
- References (1 to many)
- Formulas (1 to many)

### Chapter has one Performance Target
- Target pages, figures count, code examples count

### Code Example references
- Dependencies (external packages)
- PerformanceSpec (for performance characteristics)
- References (for algorithm sources)

### Figure belongs to one Chapter
- But may be referenced from multiple locations within the chapter

### Formula belongs to one Chapter
- But may be used in multiple sections of the chapter

### Practice Problem belongs to one Chapter
- May reference multiple concepts from the chapter

## State Transitions

### Chapter States
- `draft`: Initial creation, content being developed
- `review`: Content complete, undergoing review
- `revised`: Review feedback incorporated
- `final`: Approved for publication
- `published`: Included in final textbook

### Code Example States
- `implementation`: Code being written
- `testing`: Unit tests being developed and validated
- `validated`: All tests pass, performance verified
- `integrated`: Included in chapter content

## Constraints & Business Rules

### Content Constraints
- Each chapter must have 3-5 learning objectives
- Each chapter must include 2-5 figures (except Ch 1-2: 2-3 figures)
- Each chapter must include 1-4 code examples (except Ch 1-2: 0-2 examples)
- Each chapter must include 3-5 practice problems
- Total textbook must be 80-120 pages

### Technical Constraints
- All code must execute in Python 3.10+ clean environment
- All figures must be ≥300 DPI (raster) or SVG (vector)
- All formulas must include variable definitions with units
- All references must follow APA 7 format
- All practice problems must have verifiable solutions

### Quality Constraints
- Flesch-Kincaid Grade Level: 10-12
- Plagiarism score: <15%
- All code examples must include unit tests
- All mathematical content must be dimensionally consistent