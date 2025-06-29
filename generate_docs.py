#!/usr/bin/env python3
"""
Documentation Generator for MowerArduino

This script parses C++ header files and generates Markdown documentation.
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Optional, Tuple

class DocumentationParser:
    """Parser for C++ header files with Doxygen-style comments."""
    
    def __init__(self):
        self.current_comment = ""
        self.comment_stack = []
    
    def extract_comments(self, content: str) -> Dict[str, str]:
        """Extract all comments from the file content."""
        comments = {}
        
        # Match Doxygen-style comments
        pattern = r'(/\*\*[^*]*\*+(?:[^/*][^*]*\*+)*/|//[^\n]*)'
        
        for match in re.finditer(pattern, content):
            comment = match.group(1)
            # Get the position where the comment ends
            end_pos = match.end()
            # Find the next non-whitespace character after the comment
            next_char_pos = end_pos
            while next_char_pos < len(content) and content[next_char_pos].isspace():
                next_char_pos += 1
            
            # Store the comment with its position
            comments[next_char_pos] = self.clean_comment(comment)
        
        return comments
    
    def clean_comment(self, comment: str) -> str:
        """Clean and format a comment string."""
        # Remove comment markers
        if comment.startswith('/*'):
            # Remove /* and */
            comment = comment[2:-2]
            # Remove leading * on each line
            comment = '\n'.join(line.strip(' *') for line in comment.splitlines())
        elif comment.startswith('//'):
            # Remove //
            comment = comment[2:].strip()
        
        return comment.strip()
    
    def parse_header_file(self, file_path: str) -> List[Dict]:
        """Parse a C++ header file and extract class and method documentation."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract all comments first
        comments = self.extract_comments(content)
        
        # Remove comments for parsing
        content_no_comments = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
        content_no_comments = re.sub(r'//.*$', '', content_no_comments, flags=re.MULTILINE)
        
        # Extract class definitions
        classes = []
        class_pattern = re.compile(r'class\s+(\w+)\s*[:{]')
        class_matches = list(class_pattern.finditer(content_no_comments))
        
        for i, match in enumerate(class_matches):
            class_name = match.group(1)
            class_def = {
                'name': class_name,
                'methods': [],
                'description': comments.get(match.start(), '')
            }
            
            # Find the class body
            start = match.end()
            end = content_no_comments.find('};', start)
            if end == -1 and i + 1 < len(class_matches):
                end = class_matches[i+1].start()
            
            if end == -1:
                class_body = content_no_comments[start:]
            else:
                class_body = content_no_comments[start:end]
            
            # Extract methods with their comments
            self._extract_methods(class_body, class_def, comments)
            
            classes.append(class_def)
        
        return classes
    
    def _extract_methods(self, class_body: str, class_def: Dict, comments: Dict[int, str]) -> None:
        """Extract methods from class body with their documentation."""
        # This pattern matches method declarations
        method_pattern = re.compile(r'\s*((?:virtual\s+)?(?:[\w:<>*&]+\s+)+)(\w+)\s*\(([^)]*)\)\s*(?:const\s*)?(?:=\s*0\s*)?[;{]')
        
        for match in method_pattern.finditer(class_body):
            return_type = match.group(1).strip()
            method_name = match.group(2).strip()
            params = [p.strip() for p in match.group(3).split(',') if p.strip()]
            
            # Skip constructors/destructors
            if method_name == class_def['name'] or method_name == f'~{class_def["name"]}':
                continue
            
            # Get the comment for this method if it exists
            method_comment = comments.get(match.start(), '')
            
            method_def = {
                'name': method_name,
                'return_type': return_type,
                'params': self._parse_parameters(params, method_comment),
                'description': self._extract_description(method_comment),
                'return_description': self._extract_return_description(method_comment)
            }
            
            class_def['methods'].append(method_def)
    
    def _parse_parameters(self, params: List[str], comment: str) -> List[Dict]:
        """Parse parameter list and extract their descriptions from the comment."""
        param_descriptions = {}
        
        # Extract @param descriptions
        param_matches = re.finditer(r'@param\s+(\w+)\s+(.*?)(?=@|$)', comment, re.DOTALL)
        for m in param_matches:
            param_descriptions[m.group(1)] = m.group(2).strip()
        
        # Process each parameter
        result = []
        for param in params:
            # Split into type and name
            parts = param.rsplit(' ', 1)
            if len(parts) == 2:
                param_type, param_name = parts
                # Clean up parameter name (remove &, *, =default, etc.)
                param_name = re.sub(r'[&=].*$', '', param_name).strip()
                result.append({
                    'name': param_name,
                    'type': param_type.strip(),
                    'description': param_descriptions.get(param_name, '')
                })
        
        return result
    
    def _extract_description(self, comment: str) -> str:
        """Extract the main description from a comment."""
        # Remove all @-prefixed lines
        description = re.sub(r'^\s*@.*$', '', comment, flags=re.MULTILINE)
        return description.strip()
    
    def _extract_return_description(self, comment: str) -> str:
        """Extract the return description from a comment."""
        match = re.search(r'@return\s+(.*?)(?=@|$)', comment, re.DOTALL)
        return match.group(1).strip() if match else ""

def generate_markdown(classes: List[Dict], output_dir: str) -> None:
    """Generate Markdown documentation from parsed classes."""
    os.makedirs(output_dir, exist_ok=True)
    
    # Create an index file
    with open(os.path.join(output_dir, 'README.md'), 'w', encoding='utf-8') as index_file:
        index_file.write('# API Reference\n\n')
        index_file.write('## Classes\n\n')
        
        # Sort classes alphabetically
        classes_sorted = sorted(classes, key=lambda c: c['name'])
        
        # Write class list to index
        for class_def in classes_sorted:
            index_file.write(f'- [{class_def["name"]}]({class_def["name"]}.md)\n')
        
        index_file.write('\n---\n\n')
        index_file.write('Generated by [MowerArduino Documentation Generator](./generate_docs.py)\n')
    
    # Generate documentation for each class
    for class_def in classes_sorted:
        class_name = class_def['name']
        output_path = os.path.join(output_dir, f'{class_name}.md')
        
        with open(output_path, 'w', encoding='utf-8') as f:
            # Class header with navigation
            f.write(f'[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)\n\n')
            f.write(f'# {class_name} Class\n\n')
            
            # Class description
            if class_def.get('description'):
                f.write('## Overview\n\n')
                f.write(f'{class_def["description"]}\n\n')
            
            # Methods section
            if class_def['methods']:
                f.write('## Methods\n\n')
                
                # Method index
                f.write('### Quick Reference\n\n')
                for method in sorted(class_def['methods'], key=lambda m: m['name']):
                    params = ', '.join(f"{p['type']} {p['name']}" for p in method['params'])
                    f.write(f'- [{method["return_type"]} {method["name"]}({params})](#{method["name"].lower()})\n')
                f.write('\n')
                
                # Method details
                f.write('### Method Details\n\n')
                for method in sorted(class_def['methods'], key=lambda m: m['name']):
                    # Method signature
                    params = ', '.join(f"{p['type']} {p['name']}" for p in method['params'])
                    f.write(f'#### {method["name"]}\n\n')
                    f.write('```cpp\n')
                    f.write(f'{method["return_type"]} {method["name"]}({params})')
                    if method.get('const', False):
                        f.write(' const')
                    f.write('\n```\n\n')
                    
                    # Description
                    if method['description']:
                        f.write(f'{method["description"]}\n\n')
                    
                    # Parameters
                    if method['params']:
                        f.write('**Parameters:**\n\n')
                        for param in method['params']:
                            f.write(f'- `{param["name"]}` ({param["type"]}): {param["description"] or "No description"}\n')
                        f.write('\n')
                    
                    # Return value
                    f.write('**Returns:**\n\n')
                    f.write(f'{method["return_type"]} - {method.get("return_description", "No description")}\n\n')
                    
                    f.write('---\n\n')
            
            # Add navigation footer
            f.write('---\n\n')
            f.write('[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)\n')

def main():
    # Configuration
    src_dir = 'src'
    output_dir = 'docs/api'
    
    # Create documentation directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Find all header files
    header_files = []
    for root, _, files in os.walk(src_dir):
        for file in files:
            if file.endswith('.h'):
                header_files.append(os.path.join(root, file))
    
    if not header_files:
        print(f"No header files found in {src_dir}")
        return
    
    print(f"Found {len(header_files)} header files")
    
    # Initialize parser
    parser = DocumentationParser()
    
    # Process each header file
    all_classes = []
    for header_file in header_files:
        try:
            print(f"Processing {header_file}...")
            classes = parser.parse_header_file(header_file)
            if classes:
                print(f"  Found {len(classes)} classes")
                all_classes.extend(classes)
        except Exception as e:
            print(f"Error processing {header_file}: {e}")
    
    if not all_classes:
        print("No classes found in any header files")
        return
    
    print(f"\nFound {len(all_classes)} classes total")
    
    # Generate documentation
    print(f"\nGenerating documentation in {output_dir}...")
    generate_markdown(all_classes, output_dir)
    print("\nDocumentation generation complete!")
    print(f"\nView the documentation by opening {os.path.join(output_dir, 'README.md')} in a Markdown viewer.")

if __name__ == '__main__':
    main()
