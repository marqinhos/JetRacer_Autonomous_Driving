#!/usr/bin/env python3

#
# This file is part of the repo: https://github.com/marqinhos/JetRacer_Autonomous_Driving
# If you find the code useful, please cite the Author: Marcos Fernandez Gonzalez
# 
# Copyright 2023 The JetRacer Autonomous Driving Author. All Rights Reserved.
#
# Licensed under the AGPL-3.0 License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/agpl-3.0.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ========================================================================================


class Point:

    def __init__(self, x: int, y: int) -> None:
        self.x = x
        self.y = y  


    def __str__(self) -> str:
        return f"({self.x}, {self.y})"
    

    def is_right_than(self, other_point: "Point") -> bool:
        """Function to know is self point is in a more to the right position than other point

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
    
    def zero(self) -> bool:
        """Function to check if 0,0 point

        Returns:
            bool: Return True if 0,0 point and False in the other cases
        """

        return self.x == 0 and self.y == 0


    
