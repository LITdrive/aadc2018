#################################################################################
#
# copyTemplateProject.py
# Author: Philipp Seidl
# Date: 18.07.2018
#
# What does it do?: script copies either the openCV template or normal Template
#
# usage: 	open terminal in aadcUser
#		type in following command:
#		python herComesMyFilterName _optional_argument
#		if the _optional_argument is passed or anything at all
#  		it copies the openCV Template, otherwise the templateFilter stuff
#
################################################################################

import sys
import os
import shutil

#from https://stackoverflow.com/questions/1868714/how-do-i-copy-an-entire-directory-of-files-into-an-existing-directory-using-pyth
def copytree(src, dst, symlinks=False, ignore=None):
    for item in os.listdir(src):
        s = os.path.join(src, item)
	# change d if *.h or *.cpp
	fname, ending = item.split('.')
	if(fname == 'stdafx' or fname == 'CMakeLists'):
		d = os.path.join(dst, item)
	else:
		d = os.path.join(dst, dst+'.'+ending)
        if os.path.isdir(s):
            shutil.copytree(s, d, symlinks, ignore)
        else:
            shutil.copy2(s, d)

def replaceStuff(fName, find, replace_with):
	with open(fName) as f:
		newText=f.read().replace(find, replace_with)
	with open(fName, "w") as f: #and replace it 
		f.write(newText)

filterName = sys.argv[1]

if len(sys.argv)>2: #just some argument after the filtername
	src = 'OpenCVTemplate'
else:
	src = 'TemplateFilter'

#copy the stuff
# if filtername already exists -- abort
if os.path.exists(filterName):
	print('Error: Filter Folder allready exists - operation canceled')
else:
	os.mkdir(filterName)
	copytree(src, filterName)
	#differences in the two templates openCV or normal
	if src == 'OpenCVTemplate':
		repl1 = 'opencv_template'
		repl2 = 'cOpenCVTemplate'
		repl3 = 'OpenCV Template'
		repl4 = 'OpenCVTemplate.h'
	else:
		repl1 = 'template_filter'
		repl2 = 'cTemplateFilter'
		repl3 = 'TemplateDataFilter'
		repl4 = 'TemplateFilter.h'

	# 1) add filter directory to cmake
	with open("CMakeLists.txt", "a") as f:
    		f.write("\nadd_subdirectory("+filterName+")")
	# 1.1) change cmak in subdir
	cmFileName = filterName+'/'+'CMakeLists.txt'
	replaceStuff(cmFileName, repl1, filterName.lower()+'_filter')
	replaceStuff(cmFileName, src, filterName) #replaces TemplateFilter.h and TemplateFilter.cpp

	# 2) change filterName.h
	# 2.1) add unique ID #TODO check if unique ;)
	# replace either opencv_template or template_filter

	# 2.2) replace cTemplateFilter or
	hFileName = filterName+'/'+filterName+'.h'
	replaceStuff(hFileName, repl1, filterName.lower()+'_filter')
	replaceStuff(hFileName, repl2, 'c'+filterName)

	# 3) change *.cpp file
	cppFileName = filterName+'/'+filterName+'.cpp'
	replaceStuff(cppFileName, repl4, filterName+'.h') #change include
	replaceStuff(cppFileName, repl2, 'c'+filterName) #change cTemplateFilter
	replaceStuff(cppFileName, repl3, filterName+'_cf') #Name distplayed in ADTF (_cf) for custom filter

	print('Operation was successfully - now get on and get stuff done ;)')