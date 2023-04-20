import numpy as np

######################
## A lo mejor se puede mirar de intentar predecir hacia donde va 

def point_middle(point1, point2):
    return (point1 + point2) // 2

def extract_file(data_file: np.ndarray):
    dict_4_corner = {}
    middle_line = {}
    cont = 0
    cont2 = 0

    for j in range(len(data_file)):
        row = data_file[j]
        in_line = []
        for i in range(len(row)):
            if row[i] == 1:
                is_corner = False
                try:
                    if row[i-1] == 0:
                        is_corner = True
                    elif row[i+1] == 0:
                        is_corner = True
                except:
                    if i == 0: is_corner = True
                    elif i == len(row): is_corner = True

                if is_corner:
                    dict_4_corner[cont] = [j, i]
                    in_line.append(i)
                    cont += 1
        if len(in_line) == 2:
            middle_line[cont2] = [j, point_middle(in_line[0], in_line[1])]
            cont2 += 2

    return dict_4_corner, middle_line


