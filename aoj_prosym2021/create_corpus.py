# ja: 27212~

import linecache
# linecache.getline(filename, 整数) 整数で指定した行を読み込む

with open('prosym_train_corpus.txt', 'a') as f :
    for i in range(3401) :
        py = linecache.getline('aoj_prosym2021/prosym_valid.txt', i + 1)
        ja = linecache.getline('aoj_prosym2021/prosym_valid.txt', i + 3402)
        f.write('<SOS>' + py[:-1] + '<tab>' + ja[:-1] + '<EOS>')
        f.write('\n')
