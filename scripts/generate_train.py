import os

image_files = []

os.chdir("..")
os.chdir(os.path.join("data", "train_data"))

for filename in os.listdir(os.getcwd()):
	if (filename.endswith("jpg") or filename.endswith("JPEG")):
		image_files.append("data/train_data/"+filename)

os.chdir("..")

with open("train.txt", "w") as output:
	for image in image_files:
		output.write(image)
		output.write("\n")
	output.close()

os.chdir("..")
