with open('aoj_data_txt/aoj_train.txt', 'a') as f :
    with open('aoj_data_txt/train_py.txt', 'r') as py, open('aoj_data_txt/train_ja.txt', 'r') as ja :
        for line in py :
            f.write(line)
        for line in ja :
            f.write(line)
