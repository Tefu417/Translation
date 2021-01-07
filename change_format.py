with open('new_euler_train_corpus.txt', 'a') as f :
    with open('euler_train_corpus.txt', 'r') as c :
        for line in c :
            tab = line.find('<tab>')
            f.write(line[5:tab])
            f.write('\n')
