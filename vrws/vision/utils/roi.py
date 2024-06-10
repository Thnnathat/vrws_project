from shapely.geometry import Polygon
import numpy as np

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

    def crop(self, image, roi_point):
        polygon = Polygon(np.array(roi_point))

        mask = np.where(polygon.contains(np.indices(image.shape[0], image.shape[1])))

        cropped_image = image[mask[0], mask[1]]

        return cropped_image

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