import glob
import os
import cv2

class SaveIMG:

    def __init__(self) -> None:
        """Init function. 
            -> root_path_labels (str): Path that contain all labels
            -> root_path_images (str): Path that contain all images
            -> path2save_img (str): Path for save selective images
            -> img_extension (str): Extension images
        """
        self.root_path_labels = "./data/labels/*.txt"
        self.path2save_img = "./data/images/"
        self.img_extension = ".jpg"
        self.root_path_images = "./data/all_img/"


    def __call__(self):

        for filename in glob.glob(self.root_path_labels, recursive=True):

            pre, ext = os.path.splitext(filename)
            img_name = os.path.basename(pre)
            img = cv2.imread(self.root_path_images+img_name+ self.img_extension)
            cv2.imwrite(self.path2save_img+img_name+ self.img_extension, img)

            

if __name__ == "__main__":
    SaveIMG()()