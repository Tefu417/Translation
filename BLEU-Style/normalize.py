import re
from janome.tokenizer import Tokenizer

def normalizeString(s):
    # s = re.sub(r"\n", r" ", s)
    s = re.sub(r"\t+", r" ", s)
    # + 1回以上の繰り返し
    # s = re.sub(r"<SOS>", r"", s)
    s = re.sub(r"\(", r" ( ", s)
    s = re.sub(r"\)", r" ) ", s)
    s = re.sub(r"\{", r" { ", s)
    s = re.sub(r"\}", r" } ", s)
    s = re.sub(r"\[", r" [ ", s)
    s = re.sub(r"\]", r" ] ", s)
    s = re.sub(r"\:", r" : ", s)
    s = re.sub(r"\,", r" , ", s)
    s = re.sub(r"\"", r" \" ", s)
    s = re.sub(r"\'", r" ' ", s)
    s = re.sub(r",", r" , ", s)
    s = re.sub(r"\.", r" . ", s)
    s = re.sub(r" {2,}", r" ", s)
    # {x, y} x回以上、y回以下の繰り返し
    return s

def JSentence(sentence):
    tokens = list(Tokenizer().tokenize(sentence, wakati = True))
    return tokens

with open('BLEU-Style/normalize-all-ja.txt', 'w') as newfile :
    with open('BLEU-Style/all-ja.txt', 'r') as f :
        l = f.readlines()
        for i in l :
            line = normalizeString(i)

            for j in JSentence(line) :
                newfile.write(j)
                newfile.write(' ')

            newfile.write('\n')

# with open('BLEU-Style/normalize-all-py.txt', 'w') as newfile :
#     with open('BLEU-Style/all-py.txt', 'r') as f :
#         l = f.readlines()
#         for i in l :
#             line = normalizeString(i)

#             newfile.write(line)