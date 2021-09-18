import shutil, random, os

dirpath = 'path/to/dataset/'

names = os.listdir(dirpath)
img_names = []
for name in names:
	if(name.endswith('.jpg')):
		img_names.append(name)
txt_names = []

req_text_names = []
for txt_f in names:
	if(txt_f.endswith('.txt')):
		req_text_names.append(txt_f)
for val in img_names:
	start = val.split('.')[0]
	txt_names.append(start+'.txt')

existing_set = set(req_text_names)
i = 1
for item in txt_names:
	if item in existing_set:
		continue
	else:
		name = item.split('.')[0]
		im_nam = name+'.jpg'
		os.remove(dirpath+im_nam)
		print(i)
		print('\n')
		i = i+1

