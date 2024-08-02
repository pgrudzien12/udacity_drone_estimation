# read Graph1.txt and calculate std and mean of the data
# output the result to Graph1_stats.txt
def calc_stats(file):
    with open(file, 'r') as f:
        data = f.readlines()[1:]
        # this is csv file so we can split by comma
        data = [float(x.split(',')[1]) for x in data]
        # calc std and mean
        mean = sum(data) / len(data)
        std = (sum([(x - mean) ** 2 for x in data]) / len(data)) ** 0.5


    output_file = file.split('.')[0] + '_stats.txt'
    with open(output_file, 'w') as f:
        f.write('mean: {}\n'.format(mean))
        f.write('std: {}\n'.format(std))

calc_stats('Graph1.txt')
calc_stats('Graph2.txt')
