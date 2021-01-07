with open('Seq2Seq/euler-train.txt', 'a') as newfile :
    for line in set(open('Seq2Seq/Duplicated-euler-train.txt').readlines()) :
        newfile.write(line)
