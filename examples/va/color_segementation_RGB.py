import cv2
import matplotlib.pyplot as plt
import numpy as np


class VASegmentation:
    """Class to segmentate using VA
    """

    def __init__(self, shows: bool=False) -> None:
        ## Update values of dictionary to work in RGB space
        self.color_dict_HSV = {'black': [(0, 0, 0), (180, 255, 30)],
                                'white': [(0, 0, 231), (180, 18, 255)],
                                'orange': [(180, 80, 0), (255, 179, 93)]}
        ## (255, 89, 0) high - (255, 149, 0)
        ## (241, 156, 33) (255, 144, 32)
        ## (1, 190, 200), (18, 255, 255)
        self.__show = shows

    def get_lines(self, img: np.ndarray, color: str="orange") -> list:
        """Function to get all lines by means of color segmentation

        Args:
            img (np.ndarray): Image to process
            color (str, optional): Color you want to segment. Defaults to "orange".

        Returns:
            list: List of list, with 4 points of the square like as: [[(x, y), (x+w, y), ...], [], ...]
        """
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(img, self.color_dict_HSV[color][0], self.color_dict_HSV[color][1])
        if self.__show:
            self.__shows_img(mask)
        result = cv2.bitwise_and(img, img, mask=mask)
        if self.__show:
            self.__shows_img(result)
        mask_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        result_list = []
        if len(mask_contours) != 0:
            for mask_contour in mask_contours:
                if cv2.contourArea(mask_contour) > 500:
                    x, y, w, h = cv2.boundingRect(mask_contour)
                    ##                   Pt 1     Pt 2      Pt 3       Pt 4
                    result_list.append([(x, y), (x+w, y), (x, y+h), (x+w, y+h)])
                    if self.__show:
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.circle(img,(x, y), 10, (0,255,0), -1)
                        cv2.circle(img,(x+w, y), 10, (0,255,0), -1)
                        cv2.circle(img,(x, y+h), 10, (255,0,0), -1)
                        cv2.circle(img,(x+w, y+h), 10, (255,0,0), -1)

                        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 3) 
                        cv2.putText(img,'Black',(50,50), font, 1,(0,0,0),2)
        if self.__show:
            self.__shows_img(img)

    def __shows_img(self, img: np.ndarray) -> None:
        """Function to show a image, using matplotlib

        Args:
            img (np.ndarray): Image you want to show
        """
        plt.imshow(img)
        plt.show()
        
        

if __name__ == "__main__":
    img = cv2.imread('/home/marcos/TFG/JetRacer_Autonomous_Driving/va/data/image_5.jpg')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    ### Adjust the brightness and contrart of image
    need_adjust = False
    if need_adjust:
        # Defining alpha and beta:
        alpha = 2.0   # Contrast Control [1.0-3.0]
        beta = 1    # Brightness Control [0-100]
        img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

    seg = VASegmentation(shows=True)
    seg.get_lines(img)
