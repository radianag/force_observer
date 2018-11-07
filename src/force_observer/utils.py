import numpy as np

def save_csv_data(folder, name, data):
    with open(folder + name + '.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(np.size(data, 0) - 10):
            wr.writerow(data[i])
