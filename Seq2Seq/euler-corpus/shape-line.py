# 2~20.txt →　Code(tab)## Japanese
# それぞれの行を <SOS>Code<tab>Japanese<EOS> という形にする

import glob

with open('euler-corpus.txt', 'a') as newfile :

    for f in glob.glob('Seq2Seq/euler-corpus/*.txt') :
        with open(f, 'r') as oldfile :
            for line in oldfile :
                n = line.find('   ## ')
                c = line[: n]
                ja = line[n + 6 : -1]
                l = str('<SOS>' + c + '<tab>' + ja + '<EOS>')
                newfile.write(l)
                newfile.write('\n')
