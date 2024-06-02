
class InterestRegion:
    def __init__(self):
        self.__offest_x: int = 0
        self.__offest_y: int = 0
        self.__width: int = 0
        self.__height: int = 0

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
    
    