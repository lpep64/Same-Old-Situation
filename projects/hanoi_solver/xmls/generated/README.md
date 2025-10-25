# Generated XML Files

This directory contains automatically generated XML files from the MuJoCo XML Builder system.

## Contents

- `debug_generated_scene.xml` - Debug output from the XML builder
- `test_simple.xml` - Simple test scene for XML builder validation

## Notes

- These files are typically temporary and used for debugging
- Generated files can be safely deleted as they are recreated by scripts
- Consider adding `*.xml` to `.gitignore` if these are purely temporary
- Use these files to inspect the XML output when debugging the builder system

## Generated File Naming Convention

- `debug_*.xml` - Debug outputs from development
- `test_*.xml` - Test scenarios and validation
- `temp_*.xml` - Temporary files (should be cleaned up)
- `export_*.xml` - Exported scenes for external use