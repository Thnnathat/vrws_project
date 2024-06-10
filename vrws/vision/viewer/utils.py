import cv2
import numpy as np
import pyzed.sl as sl

id_colors = [(232, 176, 59),
             (175, 208, 25),
             (102, 205, 105),
             (185, 0, 255),
             (99, 107, 252)]


def render_object(object_data, is_tracking_on):
    if is_tracking_on:
        return object_data.tracking_state == sl.OBJECT_TRACKING_STATE.OK
    else:
        return (object_data.tracking_state == sl.OBJECT_TRACKING_STATE.OK) or (
                    object_data.tracking_state == sl.OBJECT_TRACKING_STATE.OFF)


def generate_color_id_u(idx):
    arr = []
    if idx < 0:
        arr = [236, 184, 36, 255]
    else:
        color_idx = idx % 5
        arr = [id_colors[color_idx][0], id_colors[color_idx][1], id_colors[color_idx][2], 255]
    return arr


def draw_vertical_line(left_display, start_pt, end_pt, clr, thickness):
    n_steps = 7
    pt1 = [((n_steps - 1) * start_pt[0] + end_pt[0]) / n_steps
        , ((n_steps - 1) * start_pt[1] + end_pt[1]) / n_steps]
    pt4 = [(start_pt[0] + (n_steps - 1) * end_pt[0]) / n_steps
        , (start_pt[1] + (n_steps - 1) * end_pt[1]) / n_steps]

    cv2.line(left_display, (int(start_pt[0]), int(start_pt[1])), (int(pt1[0]), int(pt1[1])), clr, thickness)
    cv2.line(left_display, (int(pt4[0]), int(pt4[1])), (int(end_pt[0]), int(end_pt[1])), clr, thickness)

def xywh_to_rectangle(roi_point: tuple[int, int, int, int], img_scale: list[float, float]):
    return (
            (int(roi_point[0] * img_scale[0]), int(roi_point[1] * img_scale[1])),
            (int((roi_point[0] + roi_point[2]) * img_scale[0]), int((roi_point[1] + roi_point[3]) * img_scale[1]))
        )

def roi_point_polygon(roi_point, img_scale):
    pts = []
    for point in roi_point:
        pts.append([int(point[0] * img_scale[0]), int(point[1] * img_scale[1])])

    return pts

def cvt(pt, scale):
    """
    Function that scales point coordinates
    """
    out = [pt[0] * scale[0], pt[1] * scale[1]]
    return out