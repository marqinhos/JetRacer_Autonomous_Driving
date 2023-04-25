#!/usr/bin/env python3

import cv2
import numpy as np
import os 
import torch

# Import utils
from utils import *

class Features_Detection:
    """Class to extrac the goal point in the image 
        Errors:
            - 1: No detection
            - 2: 
    """

    def __init__(self) -> "Features_Detection":

        ########################### IMAGE ###########################
        self.offset_right = 160 + 20 #px
        self.offset_mid = 160 + 20 #px
        self.size = [640, 480] # width, height

        ########################### DETECTIONS ###########################
        self.dict_keys_detec = {
                                "right_lane": 0,
                                "left_lane": 1,
                                "middle_line": 2,
                                "corner_line": 3}


    def run(self, result: list) -> Point:
        try:
            # return self.only_test(result)
            return self.process(result)
        
        except Exception as err:
            num_err = err.args[0]
            if num_err == 1:
                ## No detections
                raise ValueError(1)
            
            elif num_err == 2:
                ## Other error
                raise ValueError(2)

            """
            num_err = err.args[0]
            if num_err == 1:
                ## No detections
                return Point(self.size[0]//2, self.size[1])
            elif num_err == 2:
                ## Other error
                return Point(self.size[0]//2, self.size[1])
            """
    
    def only_test(self, result: list) -> Point:
        class_name = "right_lane"
        try:
            index_mask = self.__get_index_detection(result, class_name)
            mask = result[0].masks[index_mask].masks.squeeze()
            pt_centroid = self.__get_centroid_lane(mask)
            return pt_centroid
        
        except Exception as err:
            num_err = err.args[0]
            if num_err == 1:
                ## No detections 
                raise ValueError(1)
            
            raise ValueError(2)


    def process(self, result: list) -> Point:

        ##############################################################
        ##                  Check the detections                    ##
        ##############################################################
        dict_detections = {}
        list_class_num = list(self.dict_keys_detec.values())
        ## Check if lane right detection
        try:
            index = self.__get_index_detection(result, list_class_num[0])
            dict_detections[list_class_num[0]] = index
        except: pass

        ## Check if lane left detection
        try:
            index = self.__get_index_detection(result, list_class_num[1])
            dict_detections[list_class_num[1]] = index
        except: pass

        ## Check if middle line detection
        try:
            index = self.__get_index_detection(result, list_class_num[2])
            dict_detections[list_class_num[2]] = index
        except: pass

        ## Check if corners detection
        try:
            list_index = self.__get_index_detection_corner(result, list_class_num[3])
            index = self.__get_index_corner_right(list_index, result)
            dict_detections[list_class_num[3]] = index
        except: pass
        

        ##############################################################
        # 1. Detect RIGHT LINE, MIDDLE LINE
        ##############################################################
        if all(detect in [2, 3] for detect in list(dict_detections.keys())):
            mid_mask = result[0].masks[dict_detections[2]].masks.squeeze()
            mid_pt = self.__get_centroid_line_mid(mid_mask)
            corner_mask = result[0].masks[dict_detections[3]].masks.squeeze()
            corner_pt = self.__get_centroid_line_mid(corner_mask)
            return mid_pt.middle_2_point(corner_pt)
            
        ##############################################################
        # 2. Detect RIGHT LANE
        ##############################################################
        if 0 in list(dict_detections.keys()):
            lane_r_mask = result[0].masks[dict_detections[0]].masks.squeeze()
            lane_r_pt = self.__get_centroid_line_mid(lane_r_mask)
            return lane_r_pt
        
        ##############################################################
        # 3. Detect RIGHT LINE
        ##############################################################
        if 3 in list(dict_detections.keys()):
            corner_mask = result[0].masks[dict_detections[3]].masks.squeeze()
            corner_pt = self.__get_centroid_line_mid(corner_mask)
            return corner_pt
        
        ##############################################################
        # 4. Detect MIDDLE LINE
        ##############################################################
        if 2 in list(dict_detections.keys()):
            mid_mask = result[0].masks[dict_detections[2]].masks.squeeze()
            mid_pt = self.__get_centroid_line_mid(mid_mask)
            return mid_pt

        else: raise ValueError(1) # No Detections

    
    def __get_index_detection(self, result: list, class_name: str) -> int:
        tensors_detecs = result[0].boxes.cls
        class_num = self.dict_keys_detec[class_name]
        try:
            index = (tensors_detecs == class_num).nonzero(as_tuple=True)[0]
            return index
        except:
            raise ValueError(1) # Error 1: No detections
        

    def __get_index_detection_corner(self, result: list) -> list:
        tensors_detecs = result[0].boxes.cls
        class_name = "corner_line"
        class_num = self.dict_keys_detec[class_name]
        try:
            index = [(tensors_detecs == class_num).nonzero(as_tuple=True)[0][i].item() for i in range(len((tensors_detecs == class_num).nonzero(as_tuple=True)[0]))]
            return index
        except:
            raise ValueError(1) # Error 1: No detections
        

    def __get_index_corner_right(self, index: list, result: list) -> list:
        try:
            all_point = []
            for i in index:
                mask = result[0].masks[i].masks.squeeze()
                pt = self.__get_centroid_line_corner(mask)
                all_point.append(pt)
            if len(all_point) == 1:
                return index[0]
            
            else:
                if all_point[0].is_right_than(all_point[1]): return index[0]
                else: return index[1]

        except:
            raise ValueError(2)




    @staticmethod
    def __get_torch_centroid(mask: torch.Tensor) -> Point:
        """Function to get centrooid of a mask

        Args:
            mask (torch.Tensor): Mask of segment

        Returns:
            Point: Point with the x and y of corner right line centroid
        """

        y_coords, x_coords = torch.where(mask)
        ## Calculate the mean of coords
        cx = int(torch.mean(x_coords.float()))
        cy = int(torch.mean(y_coords.float()))
        return Point(cx, cy)


    @staticmethod
    def __get_numpy_centroid(mask: torch.Tensor) -> Point:
        np_mask = mask.detach().cpu().numpy().astype(np.uint8)
        # Aplica la segmentación y obtiene la máscara binaria
        _, thresh = cv2.threshold(np_mask, 0, 255, cv2.THRESH_BINARY)
        # Encuentra los contornos de la máscara
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dict_area = {}
        # Encuentra el centroide de cada contorno
        for contour in contours:
            # Calcula los momentos del contorno
            moments = cv2.moments(contour)
            area = cv2.contourArea(contour)
            dict_area[area] = moments

        max_area = max(list(dict_area.keys()))
        moment = dict_area[max_area]
        # Calcula el centroide del contorno
        cx = int(moment['m10'] / moment['m00'])
        cy = int(moment['m01'] / moment['m00'])

        return Point(cx, cy)


    def __get_centroid_lane(self, lane_mask: torch.Tensor) -> Point:
        centroid_pt = self.__get_numpy_centroid(lane_mask)
        return centroid_pt


    def __get_centroid_line_corner(self, corner_line_mask: torch.Tensor) -> Point:
        ## Get centroid of corner mask
        c_in_line = self.__get_torch_centroid(corner_line_mask)
        ## Add offset to center de point into the lane
        if c_in_line.x - self.offset_right >= 0: 
            centroid_pt = c_in_line.displace(-self.offset_right)
        else: centroid_pt = c_in_line.displace(-c_in_line.x)
        
        return centroid_pt


    def __get_centroid_line_mid(self, mid_line_mask: torch.Tensor) -> Point:
        ## Get centroid of mid mask
        c_in_line = self.__get_torch_centroid(mid_line_mask)
        ## Add offset to center de point into the lane
        if c_in_line.x + self.offset_mid <= self.size[0]: 
            centroid_pt = c_in_line.displace(self.offset_mid)
        else: centroid_pt = c_in_line.displace(-c_in_line.x)
        return centroid_pt









