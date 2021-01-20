from sklearn.model_selection import train_test_split

df = []

with open('BLEU-Style/all-ja.txt', 'r') as f :
    for i in f :
        line = f.readline()
        df.append(i[:-1])

train, valid_test = train_test_split(df, test_size=0.2, shuffle=True, random_state=123)
valid, test = train_test_split(valid_test, test_size=0.5, shuffle=True, random_state=123)

with open('BLEU-Style/train.ja', 'w') as trf :
    for tr in train :
        trf.write(tr)
        trf.write('\n')

with open('BLEU-Style/valid.ja', 'w') as vf :
    for v in valid :
        vf.write(v)
        vf.write('\n')

with open('BLEU-Style/test.ja', 'w') as tef :
    for te in test :
        tef.write(te)
        tef.write('\n')