import shutil, random, os

dirpath = 'path/to/dataset'
testDirectory = 'path/to/test-dataset directory'

names = os.listdir(dirpath)
img_names = []
for name in names:
	if(name.endswith('.jpg')):
		img_names.append(name)
filenames = random.sample(img_names,300)
txt_names = []
for val in filenames:
	start = val.split('.')[0]
	txt_names.append(start+'.txt')
print(len(filenames))
print("\n")
print(len(txt_names))
for i in range(0,300):
	src_im_path = os.path.join(dirpath,filenames[i])
	src_txt_path = os.path.join(dirpath, txt_names[i])
	shutil.move(src_im_path, testDirectory)
	shutil.move(src_txt_path,testDirectory)

'''
for fnames in filenames:
	srcpath=os.path.join(dirpath, fnames)
	shutil.move(srcpath, destDirectory)
for tfile in txt_names:
	srcpath =os.path.join(dirpath, tfile)
	shutil.move(srcpath, destDirectory)
'''
