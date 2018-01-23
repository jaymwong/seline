import os

files = [f for f in os.listdir('.') if os.path.isfile(f)]
for filename in files:
    if '.pcd' in filename:  # Check for all .pcd files
        new_fh = open("temp", "w")

        print 'Reducing: ', filename
        content = []
        with open(filename) as f:
            content = f.readlines()

        parsing_data = False
        x, y, z = [], [], []
        for line in content:

            if not parsing_data:
                new_fh.write(line)
            else:
                line = line[0:len(line)-1]
                s = line.split(' ')
                x.append(float(s[0]))
                y.append(float(s[1]))
                z.append(float(s[2]))

            if 'DATA ascii' in line:
                parsing_data = True


        x_avg = reduce(lambda x1, x2: x1 + x2, x) / len(x)
        y_avg = reduce(lambda x1, x2: x1 + x2, y) / len(y)
        z_avg = 0

        for i in xrange(0, len(x)):
            line = str(x[i]-x_avg) + ' ' + str(y[i]-y_avg) + ' ' + str(z[i]-z_avg) + '\n'
            new_fh.write(line)

        new_fh.close()
        command = 'mv temp ' + filename
        os.system(command)
print 'Done'
