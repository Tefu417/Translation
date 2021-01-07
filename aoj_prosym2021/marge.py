with open('aoj_prosym2021/prosym_valid.txt', 'a') as f :
    with open('aoj_prosym2021/valid_py.txt', 'r') as py, open('aoj_prosym2021/valid_ja.txt', 'r') as ja :
        for line in py :
            f.write(line)
        for line in ja :
            f.write(line)
