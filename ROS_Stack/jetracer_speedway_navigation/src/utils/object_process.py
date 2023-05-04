#!/usr/bin/env python3

import cv2
import numpy as np
import os 
import torch
import rospy
import yaml

# Import utils
from .utils import Point


class ObjectProcessing:
    """Class to extract the goal point in the image 
        Errors:
            - 1: No detection
            - 2: Other error
    """

    def __init__(self) -> "ObjectProcessing":

        ########################### IMAGE ###########################
        self.size = [640, 480] # width, height

        ########################### DETECTIONS ###########################
        self.dict_keys_detect = {
                                "person": 0,
                                "bicycle": 1,
                                "car": 2,
                                "motorcycle": 3,
                                "bus": 5,
                                "truck": 7,
                                "stop sign": 11,
                                "cat": 15,
                                "dog": 16,
                                "traffic light": 9}


    def run(self, result: list) -> dict:
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
            
    
    def process(self, result: list) -> dict:
        ##############################################################
        ##                  Check the detections                    ##
        ##############################################################
        dict_detections = {}
        list_class_num = list(self.dict_keys_detect.values())
        
        for i in range(list_class_num):
            try:
                index = self.__get_index_detection_corner(result, class_num=list_class_num[i])
                dict_detections[list_class_num[i]] = index
            except: pass

        ##############################################################
        ##                Calculate the centroids                   ##
        ##############################################################
        result = {}
        for cls_index in list(dict_detections.keys()):
            for index in dict_detections[cls_index]:
                detect_mask = result[0].masks[dict_detections[index]].masks.squeeze()
                detect_centroid = self.__get_torch_centroid(detect_mask)
                result.setdefault(cls_index, []).append(detect_centroid)

        return result


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