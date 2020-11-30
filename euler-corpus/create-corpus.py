# 2~20.txt →　Code(tab)## Japanese
# それぞれの行を <SOS>Code<tab>Japanese<EOS> という形にする

# train-euler-corpus.txt から test-euler-corpus.txt へ 10件分手動で移動
# KeyError を防ぐ為、train-euler-corpus に単語があることを確認しながら移動

import glob

with open('Seq2Seq/euler-corpus.txt', 'a') as newfile :

    for f in glob.glob('Seq2Seq/euler-corpus/*.txt') :
        with open(f, 'r') as oldfile :
            for line in oldfile :
                n = line.find('   ## ')
                c = line[: n]
                ja = line[n + 6 : -1]
                l = str('<SOS>' + c + '<tab>' + ja + '<EOS>')
                newfile.write(l)
                newfile.write('\n')
