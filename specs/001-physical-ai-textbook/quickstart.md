# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Date**: 2025-12-05
**Feature**: 001-physical-ai-textbook

## Getting Started

This guide helps you get up and running with the Physical AI & Humanoid Robotics textbook content, whether you want to:
- Read the textbook
- Run the code examples
- Contribute to the content
- Build the PDF

## Prerequisites

### For Reading
- Web browser (to read online version) OR PDF reader
- Basic understanding of linear algebra and calculus
- Programming background (Python preferred)

### For Running Code Examples
- Python 3.10 or higher
- pip package manager
- Git (for cloning the repository)

### For Building PDF
- Python 3.10+
- LaTeX distribution (e.g., TeX Live, MiKTeX)
- Pandoc
- Git

## Installation & Setup

### 1. Clone the Repository
```bash
git clone https://github.com/[your-repo]/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Set up Python Environment
```bash
# Create virtual environment
python -m venv textbook-env
source textbook-env/bin/activate  # On Windows: textbook-env\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Install Requirements
Create a `requirements.txt` file with:
```
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.5.0
opencv-python>=4.5.0
pybullet>=3.2.0
pytest>=7.0.0
```

## Running Code Examples

### Basic Example
```bash
# Navigate to a specific chapter's code
cd code/ch04_kinematics/

# Run a specific example
python forward_kinematics.py
```

### All Examples
```bash
# Run validation script to test all examples
python -m pytest tests/code_validation/
```

## Building the Textbook (PDF)

### 1. Install LaTeX
- **Windows**: Install MiKTeX or TeX Live
- **macOS**: Install MacTeX or BasicTeX
- **Linux**: Install texlive packages

### 2. Install Pandoc
Visit [pandoc.org](https://pandoc.org/installing.html) for installation instructions.

### 3. Build the PDF
```bash
# Run the build script
bash build/build_pdf.sh

# Or run manually:
pandoc --from markdown \
       --to pdf \
       --output textbook.pdf \
       --bibliography build/references.bib \
       --csl build/apa7.csl \
       --template build/template.tex \
       --pdf-engine=xelatex \
       chapters/*/chapter.md
```

## Chapter Structure

Each chapter follows this template structure:

```
chapters/chXX_title/
├── chapter.md          # Main content (theory, math, text)
├── figures/            # Chapter-specific figures
├── code_examples/      # Chapter-specific code files
├── practice_problems/  # Exercise files with solutions
└── references.md       # Chapter-specific references
```

## Code Example Template

All code examples follow this template:

```python
"""
Module: descriptive_name.py
Chapter: X - Chapter Name
Author: [Name]
Date: [YYYY-MM-DD]
License: MIT

Description:
    Brief 2-3 sentence description of what this code does.

Requirements:
    - numpy>=1.21.0
    - scipy>=1.7.0

Performance:
    - Time Complexity: O(n)
    - Space Complexity: O(1)
    - Execution Time: <Xms for typical input

Reference:
    Author (Year). Title. Publisher.
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

def function_name(
    param1: np.ndarray,
    param2: float,
    optional_param: bool = True
) -> np.ndarray:
    """
    One-line summary of function purpose.

    Longer description if needed. Explain algorithm, assumptions,
    and any important notes.

    Args:
        param1: Description with units (e.g., meters, radians)
        param2: Description with constraints (e.g., must be > 0)
        optional_param: Description with default behavior

    Returns:
        Description of return value with shape/units

    Raises:
        ValueError: When invalid input detected
        RuntimeError: When algorithm fails to converge

    Example:
        >>> # Provide minimal working example
        >>> result = function_name(np.array([1, 2, 3]), 0.5)
        >>> print(result)
        [expected output]

    Reference:
        Craig, J. J. (2005). Introduction to Robotics (3rd ed.).
    """
    # Input validation
    if param2 <= 0:
        raise ValueError(f"param2 must be positive, got {param2}")

    # Implementation with clear comments
    result = param1 * param2

    return result

if __name__ == "__main__":
    # Runnable example with expected output
    print("Testing function_name...")
    # Test cases here
```

## Validation Commands

### Run All Code Tests
```bash
pytest tests/code_validation/ -v
```

### Check Plagiarism
```bash
# Use Turnitin or similar service
# Target: <15% similarity score
```

### Check Readability
```bash
# Use readability tool
# Target: Grade Level 10-12
```

### Validate Citations
```bash
# Check all citations follow APA 7 format
# Verify all cited sources exist in references.bib
```

## Common Tasks

### Add a New Chapter
1. Create directory: `chapters/chXX_title/`
2. Add content following template
3. Add figures to `figures/` subdirectory
4. Add code examples to `code_examples/` subdirectory
5. Update TOC in main document

### Add a New Code Example
1. Follow the code template above
2. Include type hints and docstrings
3. Add unit tests in `tests/code_validation/`
4. Reference in appropriate chapter

### Add a New Figure
1. Create SVG or PNG (300+ DPI) in appropriate figures directory
2. Add APA 7 compliant caption
3. Include source file for future edits
4. Reference in appropriate chapter

## Troubleshooting

### Python Environment Issues
- Ensure Python 3.10+ is installed
- Check virtual environment is activated
- Reinstall packages if needed: `pip install -r requirements.txt`

### Code Examples Don't Run
- Check all dependencies are installed
- Verify Python version compatibility
- Look for import errors or missing modules

### PDF Build Fails
- Ensure LaTeX is properly installed
- Check that Pandoc is in PATH
- Verify all required files exist

## Support

For questions or issues:
- Check the [Issues](https://github.com/[repo]/issues) page
- Create a new issue for bugs or feature requests
- Contact the maintainers via [email/other contact method]