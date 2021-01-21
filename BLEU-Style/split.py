from sklearn.model_selection import train_test_split

df = []

with open('BLEU-Style/normalize-all-py.txt', 'r') as f :
    l = f.readlines()
    for i in l :
        df.append(i[:-1])

train, valid_test = train_test_split(df, test_size=0.2, shuffle=True, random_state=123)
valid, test = train_test_split(valid_test, test_size=0.5, shuffle=True, random_state=123)

with open('BLEU-Style/train.py', 'w') as trf :
    for tr in train :
        trf.write(tr)
        trf.write('\n')

with open('BLEU-Style/valid.py', 'w') as vf :
    for v in valid :
        vf.write(v)
        vf.write('\n')

with open('BLEU-Style/test.py', 'w') as tef :
    for te in test :
        tef.write(te)
        tef.write('\n')