import re

s = "HELLO WORLD world "
t = "HELLO WORLD"
u = "#akjsdhg"

# this will find all words in sequence until a non-word is encountered
i = 0
for w in re.finditer(r"(\w+)\s*", s):
    print("word ",i,w.group(1))
    i += 1

# this will find all words in sequence until a non-word is encountered
i = 0
for w in re.finditer(r"(\w+)\s*", t):
    print("word ",i,w.group(1))
    i += 1

# this will find all words in sequence until a non-word is encountered
i = 0
for w in re.finditer(r"(\w+)\s*", u):
    print("word ",i,w.group(1))
    i += 1


s = "HELLO WORLD world "
x = s.split(" ")
print(x)
# output is ['HELLO', 'WORLD', 'world', '']

x = t.split(" ")
print(x)
# output is ['HELLO', 'WORLD']

a = "CREATE TABLE tbl_1 (a1 int, a2 varchar(20));"
p = re.compile(r"(CREATE\s+TABLE)\s+(\w+)\s+\((\w+)\s+(\w+),\s+(\w+)\s+(\w+\(\d+\))")
m = p.match(a)
for g in m.groups():
    print(g)

p = re.compile(r"(\w+)\s(\w+)\s(\w+)\s\((.*)\)\;\s*")
m = p.match(a)
for g in m.groups():
    print(g)

s = "CREATE DATABASE HELLO;"
p = re.compile(r"CREATE DATABASE (\w+);\s*")
m = p.match(s)
for g in m.groups():
    print(g)

a = [1,2,3,4]
for i in a:
    print(i)
# output is
1
2
3
4
