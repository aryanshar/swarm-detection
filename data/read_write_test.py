file_1 = open("train.txt","a")
file_2 = open("req_imgPath.txt","r")

content = file_2.readlines()

for stuff in content:
	file_1.write(stuff)
file_1.close()
file_2.close()
