# MowerArduino Documentation

This directory contains the auto-generated API documentation for the MowerArduino project.

## Documentation Structure

- `api/` - Auto-generated API documentation for all classes and methods
- `old/` - Legacy documentation (if any)
- `test/` - Test documentation

## Generating Documentation

1. Make sure you have Python 3.6+ installed
2. Run the documentation generator:
   ```bash
   python3 generate_docs.py
   ```
3. The documentation will be generated in the `docs/api/` directory

## Viewing Documentation

The documentation is in Markdown format and can be viewed in any Markdown viewer or directly on GitHub.

## Adding Documentation

To add documentation to your code, use the following format for methods:

```cpp
/**
 * @brief Brief description of the method
 * 
 * @param param1 Description of first parameter
 * @param param2 Description of second parameter
 * @return Description of return value
 */
```

For classes:

```cpp
/**
 * @brief Brief description of the class
 * 
 * Detailed description of the class functionality
 */
class MyClass {
    // ...
};
```

## Updating Documentation

1. Update the code comments in the source files
2. Run the documentation generator
3. Commit the changes to both the source files and the generated documentation
