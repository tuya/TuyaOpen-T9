#!/usr/bin/env python3
# coding=utf-8

"""Print 1 for China, 2 otherwise. Logic lives in util.get_country_code."""

try:
    from tools.util import get_country_code
except ImportError:
    from util import get_country_code


if __name__ == "__main__":
    print(1 if get_country_code() == "China" else 2)
