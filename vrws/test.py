# # Python program to explain 
# # cv2.polylines() method 

# import cv2
import numpy as np

# # path
# path = 'E:/Computer Engineering/Computer Engineering Project/vrws_project/vrws/view.jpg'

# # Reading an image in default
# # mode
# image = cv2.imread(path)

# # Window name in which image is
# # displayed
# window_name = 'Image'

# # Polygon corner points coordinates
# pts = np.array([[25, 70], [25, 160], 
# 				[110, 200], [200, 160]], np.int32)

# pts = pts.reshape((-1, 1, 2))

# isClosed = True

# # Blue color in BGR
# color = (255, 0, 0)

# # Line thickness of 2 px
# thickness = 2

# # Using cv2.polylines() method
# # Draw a Blue polygon with 
# # thickness of 1 px
# image = cv2.polylines(image, [pts], 
# 					isClosed, color, thickness)

# # Displaying the image
# while(1):
	
# 	cv2.imshow('image', image)
# 	if cv2.waitKey(20) & 0xFF == 27:
# 		break
		
# cv2.destroyAllWindows()

def test_1():
    a = np.random.randint(0,10,(100,100))

    x = np.linspace(-1,5.5,100) # tried to mimic your data boundaries
    y = np.linspace(8,16,100)
    xx, yy = np.meshgrid(x,y)

    m = np.all([yy > xx**2, yy < 10* xx, xx < 4, yy > 9], axis = 0)

    print(m)

def test_2():
    where = np.where([[True, False], [True, True]],
         [[1, 2], [3, 4]],
         [[9, 8], [7, 6]])
    print(where)

    
test_2
    