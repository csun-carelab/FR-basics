#!/usr/bin/env python
"""Quick headless test to verify gripper functionality"""

import sys
import os

# Temporarily override GUI setting
import first_example as fe

# Override GUI to False for headless testing
fe.GUI = False

# Run the simulation
if __name__ == "__main__":
    fe.main()
