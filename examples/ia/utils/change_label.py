import glob
import os

class ChangeLabels:
    """Class to change YOLO *.txt labels 
    """

    def __init__(self, labels2change: dict = {0: 10}) -> None:
        """Funciton for init class

        Args:
            labels2change (dict(int), optional): Dictionary what the key is the label to change and the value is the new label. Defaults to {0: 10}.
        """
        self.root_path = "./data/labels/train/*.txt"
        self.labels2change = labels2change

    def __call__(self) -> None:

        for filename in glob.glob(self.root_path, recursive=True):
            with open(filename, 'r') as f:
                original_lines = f.readlines()
                output_line = ""
                for line in original_lines:
                    line_split = line.split(" ")
                    if int(line_split[0]) in self.labels2change.keys():
                        line_split[0] = self.labels2change[int(line_split[0])]
                        line = " ".join(line_split)
                    output_line += line
                #print(output_line)
                
                with open(filename, 'w') as out:
                    out.write(output_line)

if __name__ == "__main__":

    ChangeLabels()()