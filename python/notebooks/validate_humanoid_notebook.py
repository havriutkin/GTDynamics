#!/usr/bin/env python3
"""
Validation script for the humanoid walking MuJoCo notebook.

This script validates the notebook structure and checks for common issues.
It does NOT require GTDynamics to be installed.
"""

import json
import sys
from pathlib import Path

def validate_notebook(notebook_path):
    """Validate the notebook structure and content."""
    print(f"Validating notebook: {notebook_path}")
    print("-" * 60)
    
    # Load notebook
    try:
        with open(notebook_path, 'r') as f:
            nb = json.load(f)
    except Exception as e:
        print(f"❌ ERROR: Failed to load notebook: {e}")
        return False
    
    # Check basic structure
    if 'cells' not in nb:
        print("❌ ERROR: Notebook missing 'cells' key")
        return False
    
    cells = nb['cells']
    print(f"✓ Notebook has {len(cells)} cells")
    
    # Count cell types
    markdown_cells = sum(1 for c in cells if c['cell_type'] == 'markdown')
    code_cells = sum(1 for c in cells if c['cell_type'] == 'code')
    print(f"✓ {markdown_cells} markdown cells, {code_cells} code cells")
    
    # Check for required imports
    required_imports = [
        'import numpy',
        'import gtdynamics',
        'import gtsam',
        'import matplotlib',
        'import mujoco'
    ]
    
    all_code = []
    for cell in cells:
        if cell['cell_type'] == 'code':
            all_code.extend(cell['source'])
    
    code_text = ''.join(all_code)
    
    print("\nChecking imports:")
    for imp in required_imports:
        if imp in code_text:
            print(f"  ✓ {imp}")
        else:
            print(f"  ⚠ {imp} - not found or optional")
    
    # Check for key concepts
    key_concepts = [
        ('Robot loading', 'CreateRobotFromFile'),
        ('Walking phases', 'Phase'),
        ('Walk cycle', 'WalkCycle'),
        ('Trajectory', 'Trajectory'),
        ('Optimization', 'optimize'),
        ('MuJoCo viewer', 'mujoco.viewer'),
    ]
    
    print("\nChecking key concepts:")
    for name, keyword in key_concepts:
        if keyword in code_text:
            print(f"  ✓ {name} ({keyword})")
        else:
            print(f"  ✗ {name} ({keyword}) - NOT FOUND")
    
    # Check for proper structure
    has_markdown_intro = cells[0]['cell_type'] == 'markdown'
    has_summary = any('summary' in ''.join(c.get('source', [])).lower() 
                     for c in cells if c['cell_type'] == 'markdown')
    
    print("\nStructure checks:")
    print(f"  {'✓' if has_markdown_intro else '✗'} Starts with markdown introduction")
    print(f"  {'✓' if has_summary else '✗'} Contains summary section")
    
    # Count sections (markdown headers)
    sections = []
    for cell in cells:
        if cell['cell_type'] == 'markdown':
            for line in cell['source']:
                if line.startswith('##'):
                    sections.append(line.strip())
    
    print(f"\nFound {len(sections)} sections:")
    for section in sections[:10]:  # Show first 10
        print(f"  - {section}")
    if len(sections) > 10:
        print(f"  ... and {len(sections) - 10} more")
    
    print("\n" + "=" * 60)
    print("✓ Notebook validation complete!")
    print("\nTo run the notebook:")
    print("  1. Build and install GTDynamics with Python support")
    print("  2. Install requirements: pip install -r requirements_humanoid_walking.txt")
    print("  3. Launch Jupyter: jupyter notebook")
    print("  4. Open humanoid_walking_mujoco.ipynb")
    print("=" * 60)
    
    return True

def main():
    """Main function."""
    # Get notebook path
    script_dir = Path(__file__).parent
    notebook_path = script_dir / 'humanoid_walking_mujoco.ipynb'
    
    if not notebook_path.exists():
        print(f"❌ ERROR: Notebook not found at {notebook_path}")
        return 1
    
    # Validate
    success = validate_notebook(notebook_path)
    
    return 0 if success else 1

if __name__ == '__main__':
    sys.exit(main())
