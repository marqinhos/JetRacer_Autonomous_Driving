#!/usr/bin/env python3

import cv2
import numpy as np
import os 
import torch
import rospy
import yaml

# Import utils
from .utils import Point

class Features_Detection:
    """Class to extract the goal point in the image 
        Errors:
            - 1: No detection
            - 2: Other error
    """

    def __init__(self) -> "Features_Detection":
        
        ########################### YAML ###########################
        # Load config.yaml
        yaml_path = rospy.get_param('config_file')
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        ########################### IMAGE ###########################
        self.offset_right = config["navigation"]["offsetright"] #px
        self.offset_mid = config["navigation"]["offsetmid"] #px
        self.size = [640, 480] # width, height

        ########################### DETECTIONS ###########################
        self.dict_keys_detect = {
                                "right_lane": 0,
                                "left_lane": 1,
                                "middle_line": 2,
                                "corner_line": 3}


    def run(self, result: list) -> Point:
        try:
            return self.process(result)
        
        except Exception as err:
            num_err = err.args[0]
            if num_err == 1:
                ## No detections
                raise ValueError(1)
            
            elif num_err == 2:
                ## Other error
                raise ValueError(2)


    def process(self, result: list) -> Point:
        """Function to get the desired point (this point will be a point in the center of the lane). 
            For get the point use different methods. Methods implement:
                | RIGHT CORNER -- MIDDLE LINE -> Detect right corner line and middle line. Return the center point of both masks' centroids
                | RIGHT LANE -> Detect right lane. Return the centroid of lane mask
                | RIGHT CORNER  -> Detect right corner line. Return the mask's centroid adding right offset (to center in the lane)
                | MIDDLE LINE -> Detect middle line. Return the mask's centroid adding mid offset (to center in the lane)

            TODO: Methods to implement:
                | lane_middle_right -> Detections of right Lane, middle Line and right corner Line
                | lane_right -> Detections of right Lane and right corner Line
                | lane_middle -> Detections of right Lane and middle Line
                
        Args:
            result (list): Result of the prediction

        Raises:
            ValueError: Possible value error
                           - 1: No detections

        Returns:
            Point: Return the point in the middle of right lane
        """

        ##############################################################
        ##                  Check the detections                    ##
        ##############################################################
        dict_detections = {}
        list_class_num = list(self.dict_keys_detect.values())
        ## Check if lane right detection
        try:
            index = self.__get_index_detection(result, class_num=list_class_num[0])
            dict_detections[list_class_num[0]] = index
        except: pass

        ## Check if lane left detection
        try:
            index = self.__get_index_detection(result, class_num=list_class_num[1])
            dict_detections[list_class_num[1]] = index
        except: pass

        ## Check if middle line detection
        try:
            index = self.__get_index_detection(result, class_num=list_class_num[2])
            dict_detections[list_class_num[2]] = index
        except: pass

        ## Check if corners detection
        try:
            list_index = self.__get_index_detection_corner(result, class_num=list_class_num[3])
            index = self.__get_index_corner_right(list_index, result)
            dict_detections[list_class_num[3]] = index
        except: pass
        
        ##############################################################
        # 1. Detect RIGHT LINE, MIDDLE LINE
        ############################################################## 
        if all(detect in list(dict_detections.keys()) for detect in [2, 3]):
            rospy.loginfo("RIGHT CORNER -- MIDDLE LINE")
            mid_mask = result[0].masks[dict_detections[2]].masks.squeeze()
            mid_pt = self.__get_torch_centroid(mid_mask)
            corner_mask = result[0].masks[dict_detections[3]].masks.squeeze()
            corner_pt = self.__get_torch_centroid(corner_mask)
            return mid_pt.middle_2_point(corner_pt)
            
        ##############################################################
        # 2. Detect RIGHT LANE
        ##############################################################
        if 0 in list(dict_detections.keys()):
            rospy.loginfo("RIGHT LANE")
            lane_r_mask = result[0].masks[dict_detections[0]].masks.squeeze()
            lane_r_pt = self.__get_centroid_lane(lane_r_mask)
            return lane_r_pt
        
        ##############################################################
        # 3. Detect RIGHT LINE
        ##############################################################
        if 3 in list(dict_detections.keys()):
            rospy.loginfo("RIGHT CORNER")
            corner_mask = result[0].masks[dict_detections[3]].masks.squeeze()
            corner_pt = self.__get_centroid_line_corner(corner_mask)
            return corner_pt
        
        ##############################################################
        # 4. Detect MIDDLE LINE
        ##############################################################
        if 2 in list(dict_detections.keys()):
            rospy.loginfo("MIDDLE LINE")
            mid_mask = result[0].masks[dict_detections[2]].masks.squeeze()
            mid_pt = self.__get_centroid_line_mid(mid_mask)
            return mid_pt

        else: raise ValueError(1) # No Detections

    
    def __get_index_detection(self, result: list, class_name: str=None, class_num: int=None) -> int:
        """Function to get index for a class name or class num

        Args:
            result (list): Result of the prediction
            class_name (str, optional): Name of the class to take index. Defaults to None.
            class_num (int, optional): Num of the class to take index. Defaults to None.

        Raises:
            ValueError: Possible value error
                           - 1: No detections

        Returns:
            int: Return the index for the class enter
        """
        tensors_detects = result[0].boxes.cls
        if class_num is None:
            class_num = self.dict_keys_detect[class_name]
        else:
            class_num = class_num
        try:
            index = (tensors_detects == class_num).nonzero(as_tuple=True)[0].item()
            return index
        except:
            raise ValueError(1) # Error 1: No detections
        

    def __get_index_detection_corner(self, result: list, class_name: str=None, class_num: int=None) -> list:
        """Function to get all index of corners' detections

        Args:
            result (list): Result of the prediction
            class_name (str, optional): Name of the class to take index. Defaults to None.
            class_num (int, optional): Num of the class to take index. Defaults to None.

        Raises:
            ValueError: Possible value error
                           - 1: No detections

        Returns:
            list: Return a list with all index for detections
        """
        tensors_detects = result[0].boxes.cls
        if class_num is None:
            class_num = self.dict_keys_detect[class_name]
        else:
            class_num = class_num
        try:
            index = [(tensors_detects == class_num).nonzero(as_tuple=True)[0][i].item() for i in range(len((tensors_detects == class_num).nonzero(as_tuple=True)[0]))]
            return index
        except:
            raise ValueError(1) # Error 1: No detections
        

    def __get_index_corner_right(self, index: list, result: list) -> int:
        """Function to get only the index of the right corner's mask

        Args:
            index (list): List with all index for all corners detect
            result (list): Result of the prediction

        Raises:
            ValueError: Possible value error
                           - 2: Other error

        Returns:
            int: Return the index of right corner line
        """
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
        """Function to get centroid of a mask using GPU (torch)

        Args:
            mask (torch.Tensor): Mask of segment

        Returns:
            Point: Point with the x and y of the mask
        """
        y_coords, x_coords = torch.where(mask)
        ## Calculate the mean of coords
        cx = int(torch.mean(x_coords.float()))
        cy = int(torch.mean(y_coords.float()))
        return Point(cx, cy)


    @staticmethod
    def __get_numpy_centroid(mask: torch.Tensor) -> Point:
        """Function to get centroid of a mask using CPU (numpy)

        Args:
            mask (torch.Tensor): Mask of segment

        Returns:
            Point: Point with the x and y of the mask
        """
        np_mask = mask.detach().cpu().numpy().astype(np.uint8)
        # Apply segmentation
        _, thresh = cv2.threshold(np_mask, 0, 255, cv2.THRESH_BINARY)
        # Find mask' contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dict_area = {}
        # Find centroid for each contour
        for contour in contours:
            # Calculate moment for contour
            moments = cv2.moments(contour)
            area = cv2.contourArea(contour)
            dict_area[area] = moments

        max_area = max(list(dict_area.keys()))
        moment = dict_area[max_area]
        # Calculate centroid with the moment
        cx = int(moment['m10'] / moment['m00'])
        cy = int(moment['m01'] / moment['m00'])

        return Point(cx, cy)


    def __get_centroid_lane(self, lane_mask: torch.Tensor) -> Point:
        """Function to get the centroid of right lane mask

        Args:
            lane_mask (torch.Tensor): Right lane mask

        Returns:
            Point: Return the centroid point of right lane mask
        """
        centroid_pt = self.__get_torch_centroid(lane_mask)
        return centroid_pt


    def __get_centroid_line_corner(self, corner_line_mask: torch.Tensor) -> Point:
        """Function to get the centroid of a corner right mask, with add the offset to center in the lane 

        Args:
            corner_line_mask (torch.Tensor): The corner right mask

        Returns:
            Point: Return centroid point of this mask adding the offset to center in the lane
        """
        ## Get centroid of corner mask
        c_in_line = self.__get_torch_centroid(corner_line_mask)
        ## Add offset to center de point into the lane
        if c_in_line.x - self.offset_right >= 0: 
            centroid_pt = c_in_line.displace(-self.offset_right)
        else: centroid_pt = c_in_line.displace(-c_in_line.x)
        
        return centroid_pt


    def __get_centroid_line_mid(self, mid_line_mask: torch.Tensor) -> Point:
        """Function to get the centroid of a line mid mask, with add the offset to center in the lane 

        Args:
            mid_line_mask (torch.Tensor): The line mid mask

        Returns:
            Point: Return centroid point of this mask adding the offset to center in the lane
        """
        ## Get centroid of mid mask
        c_in_line = self.__get_torch_centroid(mid_line_mask)
        ## Add offset to center de point into the lane
        if c_in_line.x + self.offset_mid <= self.size[0]: 
            centroid_pt = c_in_line.displace(self.offset_mid)
        else: centroid_pt = c_in_line.displace(-c_in_line.x)
        return centroid_pt
