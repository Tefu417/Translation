with open('BLEU-Style/euler.txt', 'a') as newfile :
    for line in set(open('BLEU-Style/Duplicated-euler-train.txt').readlines()) :
        newfile.write(line)
