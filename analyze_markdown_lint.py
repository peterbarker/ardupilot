#!/usr/bin/env python3
"""Analyze markdownlint-cli2 error output files.

Usage:
    python3 analyze_markdown_lint.py <error-file> [options]

Options:
    --by-rule       Show errors grouped by rule (default summary)
    --by-file       Show errors grouped by file
    --rule RULE     Filter to a specific rule (e.g. MD036)
    --files-only    Just list files with errors (one per line)
    --clean-files   List files that were linted but had no errors
    --fixable       Show rules sorted by likely ease of automated fixing
    --json          Output as JSON
"""

import argparse
import json
import re
import sys

from collections import Counter
from collections import defaultdict

# Pattern matches lines like:
#   file.md:10 MD036/name Description [Context: "..."]
#   file.md:10:5 MD036/name Description [Context: "..."]
ERROR_RE = re.compile(
    r'^(?P<file>.+\.(?:md|markdown)):(?P<line>\d+)(?::(?P<col>\d+))?\s+'
    r'(?P<code>MD\d+)/(?P<names>\S+)\s+'
    r'(?P<desc>.+?)(?:\s+\[Context:\s+"(?P<ctx>.*)"\])?$'
)

# Header lines to skip
SKIP_RE = re.compile(
    r'^(?:markdownlint-cli2|Finding:|Linting:|Summary:|\s*$)'
)


def parse_errors(filepath):
    """Parse a markdownlint-cli2 error output file.

    Returns a list of dicts with keys:
        file, line, col, code, names, desc, ctx
    Also returns metadata dict with linter version, file count, error count.
    """
    errors = []
    metadata = {}

    with open(filepath) as f:
        for raw_line in f:
            line = raw_line.rstrip('\n')

            # Extract metadata from header
            if line.startswith('markdownlint-cli2'):
                metadata['version'] = line
                continue
            if line.startswith('Linting:'):
                m = re.search(r'(\d+)\s+file', line)
                if m:
                    metadata['files_linted'] = int(m.group(1))
                continue
            if line.startswith('Summary:'):
                m = re.search(r'(\d+)\s+error', line)
                if m:
                    metadata['total_errors'] = int(m.group(1))
                continue
            if SKIP_RE.match(line):
                continue

            m = ERROR_RE.match(line)
            if m:
                errors.append({
                    'file': m.group('file'),
                    'line': int(m.group('line')),
                    'col': int(m.group('col')) if m.group('col') else None,
                    'code': m.group('code'),
                    'names': m.group('names'),
                    'desc': m.group('desc'),
                    'ctx': m.group('ctx'),
                })
            elif line.strip():
                print(f"WARNING: unparsed line: {line}", file=sys.stderr)

    return errors, metadata


def extract_linted_files(filepath):
    """Extract the list of files from the Finding: line."""
    files = []
    with open(filepath) as f:
        for line in f:
            if line.startswith('Finding:'):
                # Files are space-separated after "Finding: "
                parts = line[len('Finding:'):].strip().split()
                for p in parts:
                    # Strip leading ./ and filter to markdown files
                    p = p.lstrip('./')
                    # Skip negation patterns like !modules/**
                    if p.startswith('!'):
                        continue
                    if p.endswith(('.md', '.markdown')):
                        files.append(p)
                break
    return files


def summary(errors, metadata):
    """Print a summary of errors."""
    rule_counts = Counter(e['code'] for e in errors)
    file_counts = Counter(e['file'] for e in errors)

    print(f"Total errors: {len(errors)}")
    print(f"Files with errors: {len(file_counts)}")
    if 'files_linted' in metadata:
        clean = metadata['files_linted'] - len(file_counts)
        print(f"Files linted: {metadata['files_linted']} ({clean} clean)")
    print()

    print("Errors by rule:")
    print(f"  {'Rule':<8} {'Count':>5}  Description")
    print(f"  {'----':<8} {'-----':>5}  -----------")
    for code, count in rule_counts.most_common():
        # Get description from first error with this code
        desc = next(e['names'] for e in errors if e['code'] == code)
        print(f"  {code:<8} {count:>5}  {desc}")
    print()

    print("Top files by error count:")
    print(f"  {'Count':>5}  File")
    print(f"  {'-----':>5}  ----")
    for filepath, count in file_counts.most_common(15):
        print(f"  {count:>5}  {filepath}")


def by_rule(errors, rule_filter=None):
    """Print errors grouped by rule."""
    grouped = defaultdict(list)
    for e in errors:
        if rule_filter and e['code'] != rule_filter:
            continue
        grouped[e['code']].append(e)

    for code in sorted(grouped):
        errs = grouped[code]
        name = errs[0]['names']
        print(f"\n{code}/{name} ({len(errs)} errors)")
        print("-" * 60)
        for e in errs:
            loc = f"{e['file']}:{e['line']}"
            if e['col']:
                loc += f":{e['col']}"
            ctx = f'  [{e["ctx"]}]' if e['ctx'] else ''
            print(f"  {loc}{ctx}")


def by_file(errors, rule_filter=None):
    """Print errors grouped by file."""
    grouped = defaultdict(list)
    for e in errors:
        if rule_filter and e['code'] != rule_filter:
            continue
        grouped[e['file']].append(e)

    for filepath in sorted(grouped):
        errs = grouped[filepath]
        rules = Counter(e['code'] for e in errs)
        rule_str = ', '.join(f"{c}x{n}" for c, n in sorted(rules.items()))
        print(f"\n{filepath} ({len(errs)} errors: {rule_str})")
        for e in errs:
            loc = f"  :{e['line']}"
            if e['col']:
                loc += f":{e['col']}"
            ctx = f'  [{e["ctx"]}]' if e['ctx'] else ''
            print(f"  {loc} {e['code']} {e['desc']}{ctx}")


def files_only(errors, rule_filter=None):
    """Print just the list of files with errors."""
    files = sorted(set(
        e['file'] for e in errors
        if not rule_filter or e['code'] == rule_filter
    ))
    for f in files:
        print(f)


def clean_files(errors, metadata, error_filepath):
    """Print files that were linted but had no errors."""
    linted = set(extract_linted_files(error_filepath))
    errored = set(e['file'] for e in errors)
    # Normalize paths (strip leading ./)
    errored_norm = set(f.lstrip('./') for f in errored)
    clean = sorted(linted - errored_norm)
    print(f"{len(clean)} clean files (out of {len(linted)} linted):")
    for f in clean:
        print(f"  {f}")


def as_json(errors, metadata):
    """Output everything as JSON."""
    rule_counts = Counter(e['code'] for e in errors)
    file_counts = Counter(e['file'] for e in errors)
    output = {
        'metadata': metadata,
        'summary': {
            'total_errors': len(errors),
            'files_with_errors': len(file_counts),
            'rules': {code: count for code, count in rule_counts.most_common()},
        },
        'errors': errors,
    }
    print(json.dumps(output, indent=2))


def main():
    parser = argparse.ArgumentParser(
        description='Analyze markdownlint-cli2 error output'
    )
    parser.add_argument('error_file', help='Path to the error output file')
    parser.add_argument('--by-rule', action='store_true',
                        help='Group errors by rule')
    parser.add_argument('--by-file', action='store_true',
                        help='Group errors by file')
    parser.add_argument('--rule', type=str,
                        help='Filter to a specific rule (e.g. MD036)')
    parser.add_argument('--files-only', action='store_true',
                        help='Just list files with errors')
    parser.add_argument('--clean-files', action='store_true',
                        help='List files linted with no errors')
    parser.add_argument('--json', action='store_true',
                        help='Output as JSON')

    args = parser.parse_args()

    # Normalize rule filter
    rule_filter = args.rule.upper() if args.rule else None
    if rule_filter and not rule_filter.startswith('MD'):
        rule_filter = 'MD' + rule_filter

    errors, metadata = parse_errors(args.error_file)

    if args.json:
        as_json(errors, metadata)
    elif args.clean_files:
        clean_files(errors, metadata, args.error_file)
    elif args.files_only:
        files_only(errors, rule_filter)
    elif args.by_rule:
        by_rule(errors, rule_filter)
    elif args.by_file:
        by_file(errors, rule_filter)
    elif rule_filter:
        by_rule(errors, rule_filter)
    else:
        summary(errors, metadata)


if __name__ == '__main__':
    main()
