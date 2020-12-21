import glob

with open('train-euler-corpus.csv', 'w') as newfile :

    with open('train-euler-corpus.txt', 'r') as oldfile :
        for line in oldfile :
            n = line.find('<tab>')
            c = line[5:n]
            ja = line[n + 5 : -6]
            l = str(c + ',' + ja)
            newfile.write(l)
            newfile.write('\n')
