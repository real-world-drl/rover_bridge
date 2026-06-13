# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""``python -m rover_bridge`` entry point."""

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
