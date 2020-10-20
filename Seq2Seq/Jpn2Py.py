# pip install torch torchvision

import unicodedata
# Unicode のテキスト処理を行うため
import string
# 一般的な文字列操作
import re
# 正規表現操作
import random
# 乱数を生成
import torch
# torch基本モジュール
import torch.nn as nn
# ネットワーク構築用
from torch import optim
# SGDを使うため
import torch.nn.functional as F
# ネットワーク用関数
import codecs

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

txt = ''