

class Point:

    def __init__(self, x: int, y: int) -> None:
        self.x = x
        self.y = y  


    def __str__(self) -> str:
        return f"({self.x}, {self.y})"
    

    def is_right_than(self, other_point: "Point") -> bool:
        """Function to know is self point is in a more to the right possition than other point

        Args:
            other_point (Point): Other point to compare

        Returns:
            bool: Return True or False if self point is more to the right than other point
        """
        return self.x > other_point.x


    def middle_2_point(self, other_point: "Point") -> "Point":
        """Function to take the middle point between self point and other point

        Args:
            other_point (Point): The other point

        Returns:
            Point: Return the new point that is between the middle of both points
        """
        return Point((self.x + other_point.x)//2, (self.y + other_point.y)//2)
    

    def displace(self, x: int=0, y: int=0) -> "Point":
        """Function to displace "m" px and "n" px in (x, y)

        Args:
            x (int, optional): Displacement in the x axi. Defaults to 0.
            y (int, optional): Displacement in the y axi. Defaults to 0.

        Returns:
            Point: Return the self point displaced
        """
        return Point(self.x + x, self.y + y)


    
