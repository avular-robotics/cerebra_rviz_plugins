import os
import xml.etree.ElementTree as ET
import json
import argparse

parser = argparse.ArgumentParser(description="Get all the packages in the repository")
parser.add_argument("--repo_dir", type=str, help="Repository directory", default=os.getcwd())
args = parser.parse_args()

# Iterate the repo and get all the available packages based on the package.xml
for root, dir, files in os.walk(args.repo_dir):
    for filename in files:
        if filename == "package.xml":
            full_file_path = os.path.join(root, filename)
            tree = ET.parse(full_file_path)
            xml_root = tree.getroot()

            for element in xml_root.iter("name"):
                package_name = element.text
                print(f"{package_name} {root}")
            break

