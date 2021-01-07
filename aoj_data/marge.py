with open('aoj_data_txt/aoj_valid.txt', 'a') as f :
    with open('aoj_data_txt/valid_py.txt', 'r') as py, open('aoj_data_txt/valid_ja.txt', 'r') as ja :
        for line in py :
            f.write(line)
        for line in ja :
            f.write(line)
