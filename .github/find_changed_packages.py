import os
import xml.etree.ElementTree as ET
import json
import git
import argparse
import glob
import sys

parser = argparse.ArgumentParser(description='Find the changed packages in a pull request')
parser.add_argument('--exclude', type=str, nargs='+', help='Pattern to exclude from the changed packages', default=list())
parser.add_argument('--exclude-docs', action='store_true', help='Exclude documentation changes')
parser.add_argument('base_ref', type=str, help="Base reference to compare against")
parser.add_argument('head_ref', type=str, help="Head reference to compare from")
args = parser.parse_args()

if args.exclude_docs:
    args.exclude += ['**/docs/**', '**/*.md', '**/*.MD', '**/doc/**', '**/CODEOWNERS', 'CODEOWNERS']

repo_dir = os.getcwd()
sys.stderr.write(
    f"Running script on repo directory: {os.getcwd()}, target branch {args.base_ref}, pull request branch: {args.head_ref}\n")
# Fetch all the latest changes
repo = git.Repo(repo_dir)
repo.git.fetch()

# Get the changed files dirs between the pull request and main
diff = repo.git.diff("--name-only", args.base_ref, args.head_ref)

changed_files = diff.splitlines()

# Exclude files based on the exclude patterns
if args.exclude:
    for exclude_pattern in args.exclude:
        changed_files = [file for file in changed_files if not glob.fnmatch.fnmatch(file, exclude_pattern)]

# Iterate the repo and get all the available packages based on the package.xml
packages = set()
for root, dir, files in os.walk(repo_dir):
    for filename in files:
        if filename == 'package.xml':
            full_file_path = os.path.join(root, filename)
            tree = ET.parse(full_file_path)
            root = tree.getroot()

            for element in root.iter('name'):
                package_name = element.text
                packages.add(package_name)
            break


packages_to_test = set()
for package in packages:
    for file_path in changed_files:
        if package in file_path:
            packages_to_test.add(package)

json_output = json.dumps(list(packages_to_test))

sys.stderr.write(f"I found these changed packages: {json_output}\n")

# Write changed packages to stdout
sys.stdout.write(" ".join(packages_to_test))
