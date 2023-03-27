import glob
import os


class SeparateData:

    def __init__(self, root_path: str="./datados", save_path: str = "./new_datados", img_ext: str = ".jpg", labels_need: list = ["0", "1", "2", "3", "5", "9", "11"]) -> None:
        """Function to separate data initialiation

        Args:
            root_path (str, optional): Root path of the all data folder. Defaults to "./datados".
            save_path (str, optional): Path where new data save it. Defaults to "./new_datados".
            img_ext (str, optional): Image extension i.e. ".jpg", ".png", ".jpeg". Defaults to ".jpg".
            labels_need (list, optional): Labels to want to replace. Defaults to ["0", "1", "2", "3", "5", "9", "11"].
        """
        self.root_path_labels = root_path + "/labels/train/*.txt"
        self.root_path_images = root_path + "/images/train/"
        self.path2save_labels = save_path + "/labels/train/"
        self.path2save_img = save_path + "/images/train/"
        self.img_extension = img_ext
        self.labels_need = labels_need

        self.all_file_name = []

    def __call__(self):
        """Void
        """

        for filename in glob.glob(self.root_path_labels, recursive=True):
            print(filename)
            save_labels = False
            with open(filename, 'r') as f:
                original_lines = f.readlines()
                output_line = ""
                for line in original_lines:
                    line_split = line.split(" ")
                    if line_split[0] in self.labels_need:
                        save_labels = True
                        output_line += line

                if save_labels:
                    file_name = os.path.basename(f.name)
                    try:
                        if not os.path.exists(self.path2save_img):
                            os.makedirs(self.path2save_labels)
                    except:pass
                    with open(self.path2save_labels+file_name, 'w') as out:
                        out.write(output_line)

                    pre, ext = os.path.splitext(filename)
                    img_name = os.path.basename(pre)

                    
                    with open(self.root_path_images+img_name+ self.img_extension) as img:
                        img_name_ext = os.path.basename(img.name)
                        try:
                            if not os.path.exists(self.path2save_img):
                                os.makedirs(self.path2save_img)
                        except: pass
                        with open(self.path2save_img+img_name_ext, 'w') as out:
                            out.write(output_line)


if __name__ == "__main__":

    SeparateData()()
