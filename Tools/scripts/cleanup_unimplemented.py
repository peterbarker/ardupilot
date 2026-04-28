#!/usr/bin/env python3
"""
Tool to identify and remove unimplemented method declarations.

For each candidate:
1. Remove the declaration from the header
2. Attempt to build for CubeOrangePlus and SITL
3. If both succeed, commit the removal
4. If either fails, restore and skip
"""

import os
import re
import sys
import subprocess
from pathlib import Path


class UnimplementedCleaner:
    def __init__(self, repo_root):
        self.repo_root = repo_root
        os.chdir(repo_root)
        self.candidates = []
        self.cleaned = 0
        self.skipped = 0
        self.failed = 0

    def get_class_name(self, header_path):
        """Extract primary class name from header, preferring filename match"""
        try:
            # Get the filename without extension for matching
            filename_base = os.path.basename(header_path).replace('.h', '')

            with open(header_path, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()

            # First, look for a class matching the filename
            for i, line in enumerate(lines):
                stripped = line.strip()
                if stripped.startswith('//') or stripped.startswith('#'):
                    continue

                # Match: "class ClassName" or "struct ClassName"
                # Allow for opening brace on next line
                match = re.search(r'\b(?:class|struct)\s+([a-zA-Z_][a-zA-Z0-9_]*)\b', line)
                if match:
                    class_name = match.group(1)
                    # If it matches the filename, return it immediately
                    if class_name == filename_base:
                        return class_name
                    # Otherwise, remember it as fallback
                    if not hasattr(self, '_fallback_class'):
                        self._fallback_class = class_name

            # Fall back to first class found
            return getattr(self, '_fallback_class', None)
        except Exception:
            return None

    def extract_methods(self, header_path):
        """Extract potentially unimplemented methods (not inline implementations or function calls)"""
        methods = {}
        try:
            with open(header_path, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()

            for i, line in enumerate(lines):
                # Skip lines that look like function/method calls (contain ::)
                # e.g., AP_HAL::panic(...) or Class::method(...)
                if '::' in line:
                    continue

                # Look for declaration ending with semicolon (not implementation with brace)
                if re.search(r'\)\s*(?:const)?\s*(?:override)?\s*(?:noexcept)?\s*;', line):
                    # Make sure it's not an inline implementation with body on same or next line
                    if '{' in line:
                        continue  # Inline implementation on same line

                    # Check if next line starts with { (multiline inline implementation)
                    if i + 1 < len(lines) and lines[i + 1].strip().startswith('{'):
                        continue

                    # Must look like a method declaration, not a variable declaration
                    # Should have a return type and method name
                    match = re.search(r'(\w+)\s*\([^)]*\)\s*(?:const)?\s*(?:override)?\s*(?:noexcept)?\s*;', line)
                    if match:
                        name = match.group(1)
                        # Skip keywords and macros
                        if name not in ['if', 'for', 'while', 'sizeof', 'return', 'switch', 'case']:
                            # Skip all-uppercase (macros) and leading underscore (private/special)
                            if not name.isupper() and not name.startswith('_'):
                                methods[name] = i
        except Exception:
            pass
        return methods

    def method_implemented(self, method_name, class_name, header_dir):
        """Check if method is implemented in any cpp file in the directory"""
        # Look for: ClassName::method_name(
        pattern = rf'{re.escape(class_name)}::{re.escape(method_name)}\s*\('

        # Search all .cpp files in the same directory as the header
        try:
            for cpp_file in Path(header_dir).glob('*.cpp'):
                try:
                    with open(cpp_file, 'r', encoding='utf-8', errors='ignore') as f:
                        content = f.read()
                    if re.search(pattern, content):
                        return True
                except Exception:
                    continue
        except Exception:
            pass

        return False

    def find_candidates(self):
        """Scan for unimplemented method declarations"""
        lib_root = os.path.join(self.repo_root, 'libraries')
        candidates = []

        for header_path in Path(lib_root).rglob('*.h'):
            if 'test' in str(header_path):
                continue

            class_name = self.get_class_name(str(header_path))
            if not class_name:
                continue

            methods = self.extract_methods(str(header_path))
            header_dir = os.path.dirname(str(header_path))

            for method_name in methods.keys():
                # Search in all .cpp files in the same directory
                if not self.method_implemented(method_name, class_name, header_dir):
                    candidates.append({
                        'header': str(header_path),
                        'method': method_name,
                        'class': class_name,
                        'line': methods[method_name]
                    })

        self.candidates = sorted(candidates, key=lambda x: x['header'])
        return self.candidates

    def build_target(self, target):
        """Build for a specific target"""
        try:
            print(f"  Building for {target}...", end=" ", flush=True)
            result = subprocess.run(
                f"./waf configure --board {target} >/dev/null 2>&1 && ./waf build >/dev/null 2>&1",
                shell=True,
                timeout=300,
                cwd=self.repo_root
            )
            success = result.returncode == 0
            print("✓" if success else "✗")
            return success
        except subprocess.TimeoutExpired:
            print("✗ (timeout)")
            return False
        except Exception as e:
            print(f"✗ ({e})")
            return False

    def remove_declaration(self, header_path, method_name, line_num):
        """Remove a method declaration from header"""
        with open(header_path, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()

        # Remove the line
        removed_lines = lines[:line_num] + lines[line_num+1:]

        with open(header_path, 'w', encoding='utf-8') as f:
            f.writelines(removed_lines)

    def restore_header(self, header_path, original_content):
        """Restore header from backup"""
        with open(header_path, 'w', encoding='utf-8') as f:
            f.write(original_content)

    def make_commit(self, header_path, method_name):
        """Create git commit for removal"""
        try:
            msg = f"Remove unimplemented {method_name}() declaration from {os.path.basename(header_path)}"

            subprocess.run(
                ['git', 'add', header_path],
                check=True,
                capture_output=True,
                cwd=self.repo_root
            )

            subprocess.run(
                ['git', 'commit', '-m', msg],
                check=True,
                capture_output=True,
                cwd=self.repo_root
            )
            return True
        except Exception:
            return False

    def process_candidates(self, dry_run=False, limit=None, test_mode=False):
        """Process candidates, testing and removing"""
        if not self.candidates:
            print("No candidates found. Run find_candidates() first.")
            return

        print(f"\nProcessing {len(self.candidates)} candidates...")
        if test_mode:
            print("TEST MODE - showing what would happen without building\n")
        else:
            print(f"{'Dry run' if dry_run else 'Will commit changes'}\n")

        for i, candidate in enumerate(self.candidates[:limit]):
            if i > 0 and i % 10 == 0:
                print(f"\n--- Progress: {i}/{len(self.candidates)} ---\n")

            header = candidate['header']
            method = candidate['method']
            line = candidate['line']

            rel_header = os.path.relpath(header, self.repo_root)
            print(f"[{i+1}] {rel_header}:{line+1} - {method}()", end=" ", flush=True)

            # Backup original
            with open(header, 'r', encoding='utf-8', errors='ignore') as f:
                original = f.read()

            try:
                # Remove declaration
                self.remove_declaration(header, method, line)

                if test_mode:
                    # Just show what would happen
                    print("\n  [TEST] Would test builds and potentially commit")
                    self.cleaned += 1
                    self.restore_header(header, original)
                else:
                    # Test builds (fail fast on first failure)
                    print("\n", end="")
                    cube_ok = self.build_target('CubeOrangePlus')

                    if not cube_ok:
                        print("  ✗ CubeOrangePlus build failed, skipping")
                        self.restore_header(header, original)
                        self.skipped += 1
                    else:
                        # Only test SITL if CubeOrangePlus succeeded
                        sitl_ok = self.build_target('sitl')

                        if sitl_ok:
                            if dry_run:
                                print("  [DRY RUN] Would commit removal")
                                self.cleaned += 1
                            else:
                                if self.make_commit(header, method):
                                    print("  ✓ Committed")
                                    self.cleaned += 1
                                else:
                                    print("  ✗ Commit failed")
                                    self.restore_header(header, original)
                                    self.failed += 1
                        else:
                            print("  ✗ SITL build failed, skipping")
                            self.restore_header(header, original)
                            self.skipped += 1

            except Exception as e:
                print(f"  ✗ Error: {e}")
                self.restore_header(header, original)
                self.failed += 1

    def print_summary(self):
        """Print summary statistics"""
        print(f"\n{'='*60}")
        print("Summary:")
        print(f"  Cleaned: {self.cleaned}")
        print(f"  Skipped: {self.skipped}")
        print(f"  Failed:  {self.failed}")
        print(f"{'='*60}")


if __name__ == '__main__':
    repo_root = '/home/pbarker/rc/ardupilot-claude'

    cleaner = UnimplementedCleaner(repo_root)

    dry_run = False
    limit = None
    auto_proceed = False
    test_mode = False

    # Parse arguments
    args = sys.argv[1:]
    if '--test' in args:
        test_mode = True
        args.remove('--test')
        auto_proceed = True

    if '--dry-run' in args:
        dry_run = True
        args.remove('--dry-run')
        auto_proceed = True  # Auto-proceed in dry-run mode

    if '--yes' in args:
        auto_proceed = True
        args.remove('--yes')

    if args:
        try:
            limit = int(args[0])
        except ValueError:
            print(f"Invalid limit: {args[0]}")
            sys.exit(1)

    if test_mode:
        print("TEST MODE - showing what the tool would do without actual builds\n")
    elif dry_run:
        print("DRY RUN MODE - will do actual builds but no changes will be committed\n")

    print("Scanning for unimplemented method declarations...")
    candidates = cleaner.find_candidates()
    print(f"Found {len(candidates)} candidates")

    if limit:
        print(f"Processing first {limit} candidates")

    # Show first few candidates
    print("\nFirst 10 candidates:")
    for c in candidates[:10]:
        print(f"  {os.path.relpath(c['header'], repo_root)}:{c['line']+1} - {c['method']}()")

    if auto_proceed:
        cleaner.process_candidates(dry_run=dry_run, limit=limit, test_mode=test_mode)
        cleaner.print_summary()
    else:
        response = input("\nProceed with removal attempts? (yes/no): ").strip().lower()
        if response == 'yes':
            cleaner.process_candidates(dry_run=dry_run, limit=limit, test_mode=test_mode)
            cleaner.print_summary()
        else:
            print("Cancelled.")
