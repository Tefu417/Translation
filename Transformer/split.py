# !wget http://www.manythings.org/anki/jpn-eng.zip
# !unzip jpn-eng.zip

if __name__ == "__main__":
    with open("./jpn.txt") as f, \
         open("tatoeba_jp.txt", "w") as f_jp, \
         open("tatoeba_en.txt", "w") as f_en:
        for line in f:
            line = line.strip()
            line = line.split("\t")
            f_en.write(line[0]+"\n")
            f_jp.write(line[1]+"\n")