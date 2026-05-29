"""Pytest 配置: 把 src/ 加入 sys.path 以便直接 import 包。"""

import os
import sys


HERE = os.path.dirname(__file__)
SRC = os.path.abspath(os.path.join(HERE, "..", "src"))
if SRC not in sys.path:
    sys.path.insert(0, SRC)
