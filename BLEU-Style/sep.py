with open('BLEU-Style/all-py.txt', 'w') as newfile :

    with open('BLEU-Style/euler.txt', 'r') as oldfile :
        for line in oldfile :
            n = line.find('<tab>')
            py = line[5:n]
            # jpn = line[n + 5 : -6]
            newfile.write(py)
            newfile.write('\n')
