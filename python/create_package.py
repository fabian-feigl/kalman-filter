#!/usr/bin/env python

import os
import argparse

def populate_pacakge(root):
    for root_, _, _ in os.walk(root):
        open(os.path.join(root_, "__init__.py"), "a").close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--package", type=str, required=False, help="Root folder to populate with __init__.py recursively")
    args = parser.parse_args()

    if args.package:
        populate_pacakge(args.package)
