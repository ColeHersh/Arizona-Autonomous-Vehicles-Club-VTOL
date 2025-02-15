# Author: Aarush Parvataneni
# File: differences.py
# Description: This checks for the differences in commands between the old and new PixHawk API


name1 = input("Please input file 1:")
name2 = input("Please input file 2:")

file1 = open(name1, 'r')
file2 = open(name2, 'r')

messageArr1 = []
messageArr2 = []

for line in file1:
    if line.lower().startswith("message"):
        messageArr1.append(line)

for line in file2:
    if line.lower().startswith("message"):
        messageArr2.append(line)


def checkCommand(command, msgList):
    for i in range(len(msgList)):
        if command == msgList[i]:
            msgList.pop(i)
            return True
    return False


sameList = []
file1List = []
file2List = []
for msg in messageArr1:
    if checkCommand(msg, messageArr2):
        sameList.append(msg)
    else:
        file1List.append(msg)

for msg in messageArr2:
    file2List.append(msg)


f = open("diff.txt", 'w')
f.write("File: " + name1 + " unique\n")
for msg in file1List:
    f.write(msg)
f.write("\n\n")
f.write("File: " + name2 + " unique\n")
for msg in file2List:
    f.write(msg)
f.write("\n\n")
f.write("In both\n")
for msg in sameList:
    f.write(msg)