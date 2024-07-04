from shapely.geometry import Polygon
import numpy as np
from matplotlib.path import Path
import pylab as plt
from typing import Literal

class InterestRegion:
    def __init__(self):
        self.__offest_x: int = 0
        self.__offest_y: int = 0
        self.__width: int = 0
        self.__height: int = 0

        self.__top_left: tuple[int, int] = (0, 0)
        self.__top_right: tuple[int, int] = (0, 0)
        self.__bottom_left: tuple[int, int] = (0, 0)
        self.__bottom_right: tuple[int, int] = (0, 0)

    def crop(self, image_frame, image, width, height, shape: Literal["regtangle", "polygon"]):
        image_bound = image_frame.copy()
        if shape == "regtangle":
            image_bound[self.__offest_y : self.__offest_y + self.__height, self.__offest_x : self.__offest_x + self.__width, :] = image[self.__offest_y : self.__offest_y + self.__height, self.__offest_x : self.__offest_x + self.__width, :]
            return image_bound
        if shape == "polygon":
            mask = self.poly_mask(width, height)
            image_mask = mask.reshape(height, width)
            image_bound[image_mask, :] = image[image_mask, :]
            return image_bound
    
    def poly_mask(self, width, height):
        poly_path = Path(self.transform_points_to_path(self.poly_point))

        x, y = np.mgrid[:height, :width]
        coors = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1)))

        mask = poly_path.contains_points(coors)
        return mask
    
    def transform_points_to_path(self, point_list):
        # (left, top), (left, bottom), (right, bottom), (right, top)
        p = point_list
        # (top, left), (bottom, left), (bottom, rigth), (top, right)
        points = ((p[1][1], p[1][0]), (p[0][1], p[0][0]), (p[3][1], p[3][0]), (p[2][1], p[2][0]))
        return points

    @property
    def roi_point(self):
        return (self.__offest_x, self.__offest_y, self.__width, self.__height)

    @property
    def offest_x(self):
        return self.__offest_x

    @offest_x.setter
    def offest_x(self, value):
        self.__offest_x = value
        
    @property
    def offest_y(self):
        return self.__offest_y
    
    @offest_y.setter
    def offest_y(self, value):
        self.__offest_y = value

    @property
    def width(self):
        return self.__width

    @width.setter
    def width(self, value):
        self.__width = value

    @property
    def height(self):
        return self.__height

    @height.setter
    def height(self, value):
        self.__height = value
    
    @property
    def poly_point(self):
        return (self.__top_left, self.__top_right, self.__bottom_left, self.__bottom_right)

    def set_poly_point(self, top_left_point: tuple[int, int], top_right_point: tuple[int, int], bottom_left_point: tuple[int, int], bottom_right_point: tuple[int, int]):
        self.__top_left = top_left_point
        self.__top_right = top_right_point
        self.__bottom_left = bottom_left_point
        self.__bottom_right = bottom_right_point