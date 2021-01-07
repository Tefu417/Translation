# ja: 1773~

import linecache
# linecache.getline(filename, 整数) 整数で指定した行を読み込む

with open('aoj_train_corpus.txt', 'a') as f :
    for i in range(1772) :
        py = linecache.getline('aoj_data_txt/aoj_train.txt', i + 1)
        ja = linecache.getline('aoj_data_txt/aoj_train.txt', i + 1773)
        f.write('<SOS>' + py[:-1] + '<tab>' + ja[:-1] + '<EOS>')
        f.write('\n')
