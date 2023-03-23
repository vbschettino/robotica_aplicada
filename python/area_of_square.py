#!/usr/bin/env python3

"""Calculate and prints the area of a square with side 2"""


from rectangle import Rectangle


def main():
    square = Rectangle(width=2)
    print("The perimeter of a square with side 2 is", square.get_perimeter())


if __name__ == '__main__':
    main()
