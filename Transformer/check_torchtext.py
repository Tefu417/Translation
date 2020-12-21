from torchtext import data
import sentencepiece as spm
import re
from torch.autograd import Variable
import torch
import numpy as np

# SentencePieceで単語分割
def tokenize(sentence):
    sp = spm.SentencePieceProcessor()
    sp.load('prog8k/prog8k.model')
    sentence = Normalize().normalizeString(sentence)
    return sp.EncodeAsPieces(sentence)

# 正規化
class Normalize(object):
    def normalizeString(self, s):
        s = re.sub(r"\n", r" ", s)
        s = re.sub(r"\t+", r" ", s)
        # + 1回以上の繰り返し
        s = re.sub(r"<SOS>", r"", s)
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
        s = re.sub(r" {2,}", r" ", s)
        # {x, y} x回以上、y回以下の繰り返し
        return s

def subsequent_mask(size):
    "Mask out subsequent positions."
    attn_shape = (1, size, size)
    subsequent_mask = np.triu(np.ones(attn_shape), k=1).astype('uint8')
    return torch.from_numpy(subsequent_mask) == 0

class Batch:
    "Object for holding a batch of data with mask during training."
    def __init__(self, src, trg=None, pad=0):
        self.src = src
        self.src_mask = (src != pad).unsqueeze(-2)
        if trg is not None:
            self.trg = trg[:, :-1]
            self.trg_y = trg[:, 1:]
            self.trg_mask = \
                self.make_std_mask(self.trg, pad)
            self.ntokens = (self.trg_y != pad).data.sum()

    @staticmethod
    def make_std_mask(tgt, pad):
        "Create a mask to hide padding and future words."
        tgt_mask = (tgt != pad).unsqueeze(-2)
        tgt_mask = tgt_mask & Variable(
            subsequent_mask(tgt.size(-1)).type_as(tgt_mask.data))
        return tgt_mask

BOS_WORD = '<SOS>'
EOS_WORD = '<EOS>'
BLANK_WORD = "<blank>"
MAX_LEN = 100

# Fieldオブジェクトの作成
SRC = data.Field(tokenize=tokenize, pad_token=BLANK_WORD)
TGT = data.Field(tokenize=tokenize, init_token = BOS_WORD,
                    eos_token = EOS_WORD, pad_token=BLANK_WORD)

# CSVファイルを読み込み、TabularDatasetオブジェクトの作成
train, test = data.TabularDataset.splits(
    path='Transformer/',
    train='train-euler-corpus.csv',
    test='test-euler-corpus.csv',
    format='csv',
    fields=[('src', SRC), ('trg', TGT)],
    filter_pred=lambda x: len(vars(x)['src']) <= MAX_LEN and len(vars(x)['trg']) <= MAX_LEN)

# for example in test:
#     print(example.src, example.trg)

MIN_FREQ = 2
SRC.build_vocab(train.src, min_freq=MIN_FREQ)
TGT.build_vocab(train.trg, min_freq=MIN_FREQ)

print(TGT.vocab.freqs) # 単語毎の出現回数
print(TGT.vocab.stoi) # 文字列からインデックス番号
print(TGT.vocab.itos) # インデックス番号から文字列
print(TGT.vocab.vectors) # 単語ベクトル

# print(tokenize('xを1ずつ増加させる'))
# print(tokenize('x += 1'))