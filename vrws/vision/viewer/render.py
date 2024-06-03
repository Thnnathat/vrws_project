import cv2
import numpy as np

from .utils import *
import pyzed.sl as sl

# ----------------------------------------------------------------------
#       2D LEFT VIEW
# ----------------------------------------------------------------------

class GuideLine:
    def __init__(self) -> None:
        pass

    def draw_star_line_center_frame(self, image):
        center_frame = (int(image.shape[1] // 2), int(image.shape[0] // 2))
        
        cv2.line(image, (center_frame[0], 0), (center_frame[0], image.shape[0]), (0, 0, 255), 1)
        cv2.line(image, (0, center_frame[1]), (image.shape[1], center_frame[1]), (0, 0, 255), 1)

    def draw_star_center_object(self, image, center_object, color):
        cv2.line(image, (0, center_object[1]), (image.shape[1], center_object[1]), color, 1)
        cv2.line(image, (center_object[0], 0), (center_object[0], image.shape[0]), color, 1)

    def draw_roi_rectangle(self, image, roi_point, img_scale):
        cv2_rec_point = xywh_to_rectangle(roi_point, img_scale)
        cv2.rectangle(image, cv2_rec_point[0], cv2_rec_point[1], (255, 0, 0), 1)

class Viewer():
    def __init__(self, model) -> None:
        self.x_position: float = 0.0
        self.y_position: float = 0.0
        self.z_position: float = 0.0
        self.model = model
        self.cls_names = model.names

        self.ocv_font = cv2.FONT_HERSHEY_SIMPLEX
        self.ocv_font_size = 0.5

        self.text_top_offest = 80
        self.text_left_offest = 20

    def get_image_position(self, bounding_box_image, img_scale):
        out_position = np.zeros(2)
        out_position[0] = (bounding_box_image[0][0] + (bounding_box_image[2][0] - bounding_box_image[0][0]) * 0.5) * \
                        img_scale[0]
        out_position[1] = (bounding_box_image[0][1] + (bounding_box_image[2][1] - bounding_box_image[0][1]) * 0.5) * \
                        img_scale[1]
        return out_position


    def render_2D(self, left_display, img_scale, objects, is_tracking_on):
        overlay = left_display.copy()

        line_thickness = 2
        for obj in objects.object_list:
            if render_object(obj, is_tracking_on):
                base_color = generate_color_id_u(obj.id)
                # Display image scaled 2D bounding box
                top_left_corner = cvt(obj.bounding_box_2d[0], img_scale)
                top_right_corner = cvt(obj.bounding_box_2d[1], img_scale)
                bottom_right_corner = cvt(obj.bounding_box_2d[2], img_scale)
                bottom_left_corner = cvt(obj.bounding_box_2d[3], img_scale)

                # Creation of the 2 horizontal lines
                cv2.line(left_display, (int(top_left_corner[0]), int(top_left_corner[1])),
                        (int(top_right_corner[0]), int(top_right_corner[1])), base_color, line_thickness)
                cv2.line(left_display, (int(bottom_left_corner[0]), int(bottom_left_corner[1])),
                        (int(bottom_right_corner[0]), int(bottom_right_corner[1])), base_color, line_thickness)
                # Creation of 2 vertical lines
                draw_vertical_line(left_display, bottom_left_corner, top_left_corner, base_color, line_thickness)
                draw_vertical_line(left_display, bottom_right_corner, top_right_corner, base_color, line_thickness)

                # Scaled ROI
                roi_height = int(top_right_corner[0] - top_left_corner[0])
                roi_width = int(bottom_left_corner[1] - top_left_corner[1])

                overlay_roi = overlay[int(top_left_corner[1]):int(top_left_corner[1] + roi_width)
                , int(top_left_corner[0]):int(top_left_corner[0] + roi_height)]

                overlay_roi[:, :, :] = base_color

                # Display Object label as text
                position_image = self.get_image_position(obj.bounding_box_2d, img_scale)
                
                center_object = (int(position_image[0]), int((position_image[1])))
                center_frame = (int(left_display.shape[1] // 2), int(left_display.shape[0] // 2))
                

                class_text = "class: " + str(obj.raw_label) + " | " + "Class Name: " + str(self.cls_names[obj.raw_label])
                confidence_text = "Confidence: " + str(int(obj.confidence))
                id_text = "ID: " + str(obj.id)
                text_color = (200, 255, 200, 255)
                
                guide_line = GuideLine()
                
                # guide_line.draw_star_center_object(left_display, center_object, base_color)

                left_display = cv2.circle(left_display, center_frame, 1, (255, 0, 0), 2)
                left_display = cv2.circle(left_display, center_object, 1, (0, 0, 255), 2)
                cv2.line(left_display, center_object, center_frame, base_color, 1)

                
                # ID
                text_position = (int(position_image[0] - self.text_left_offest), int(position_image[1] - 12 - self.text_top_offest))
                cv2.putText(left_display, id_text, text_position, cv2.FONT_HERSHEY_COMPLEX_SMALL, self.ocv_font_size, text_color, 1)

                # class
                text_position = (int(position_image[0] - self.text_left_offest), int(position_image[1] - self.text_top_offest))
                cv2.putText(left_display, class_text, text_position, cv2.FONT_HERSHEY_COMPLEX_SMALL, self.ocv_font_size, text_color, 1)

                # confidence
                text_position = (int(position_image[0] - 20), int(position_image[1] + 12 - self.text_top_offest))
                cv2.putText(left_display, confidence_text, text_position, cv2.FONT_HERSHEY_COMPLEX_SMALL, self.ocv_font_size, text_color, 1)

                self.x_position = obj.position[0]
                self.y_position = obj.position[1]
                self.z_position = obj.position[2]

                # Diplay Object distance to camera as text
                if np.isfinite(self.x_position and self.y_position and self.z_position):
                    text = "X: " + str(round(self.x_position, 2)) + "mm"
                    text_position = (int(position_image[0] - self.text_left_offest), int(position_image[1] + 24 - self.text_top_offest))
                    cv2.putText(left_display, text, text_position, cv2.FONT_HERSHEY_COMPLEX_SMALL, self.ocv_font_size, text_color, 1)

                    text = "Y: " + str(round(self.y_position, 2)) + "mm"
                    text_position = (int(position_image[0] - self.text_left_offest), int(position_image[1] + 36 - self.text_top_offest))
                    cv2.putText(left_display, text, text_position, cv2.FONT_HERSHEY_COMPLEX_SMALL, self.ocv_font_size, text_color, 1)

                    # text = "Z: " + str(round(self.z_position, 2)) + "mm"
                    text = "Z: " + str(round(0.6889890534449 * self.y_position + self.z_position, 2)) + "mm"
                    text_position = (int(position_image[0] - self.text_left_offest), int(position_image[1] + 48 - self.text_top_offest))
                    cv2.putText(left_display, text, text_position, cv2.FONT_HERSHEY_COMPLEX_SMALL, self.ocv_font_size, text_color, 1)

        # Here, overlay is as the left image, but with opaque masks on each detected objects
        cv2.addWeighted(left_display, 0.7, overlay, 0.3, 0.0, left_display)