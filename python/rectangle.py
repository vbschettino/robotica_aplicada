#!/usr/bin/env python3

"""Functions and classes to work with rectangle-like objects"""


def area(width, height=None):
    """Calculate the area of a rectangle."""
    if height is None:
        height = width
    return width * height


def perimeter(width, height=None):
    """Calculate the perimeter of a rectangle."""
    if height is None:
        height = width
    return 2 * (width + height)


class Rectangle:
    """Represents a rectangle-like object."""

    def __init__(self, width, height=None):
        self.width = width
        if height is None:
            self.height = width
        else:
            self.height = height

    def get_perimeter(self):
        return perimeter(self.width, self.height)

    def get_area(self):
        return area(self.width, self.height)
