#!/usr/bin/env bash

if [ "$EUID" -eq 0 ]; then
	echo "Don't run as root"
	exit
fi


read -p "Perform apt-get install (y/n): " yn
case $yn in
	[YyTt]* ) 
		sudo apt-get update
	
		# git Large File Storage
		dpkg -s git-lfs
		if [ $? -eq 1 ]; then
			curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
			sudo apt-get install -y git-lfs
			git lfs install --local
		fi

		# octave
		sudo apt-get install -y octave liboctave-dev octave-image octave-io octave-statistics octave-quaternion

		# latex
		sudo apt-get install -y texlive-science texlive-lang-polish texlive-lang-european texlive-bibtex-extra latexdiff texstudio pdflatex

		# OpenCV dependencies
		sudo apt-get install -y build-essential
		sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
		sudo apt-get install -y python-dev python-numpy python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
		;;
esac

mkdir -p opencv/build
cd opencv

# opencv 3.1
if [ ! -d "opencv-3.1.0" ]; then
	curl -L https://github.com/opencv/opencv/archive/3.1.0.tar.gz > opencv.tar.gz
	tar -xf opencv.tar.gz
	rm -f opencv.tar.gz
fi

# OpenCV 3.1 contrib
if [ ! -d "opencv_contrib-3.1.0" ]; then
	curl -L https://github.com/opencv/opencv_contrib/archive/3.1.0.tar.gz > opencv_contrib.tar.gz
	tar -xf opencv_contrib.tar.gz
	rm -f opencv_contrib.tar.gz
fi


# OpenCV build (C++ & Octave)
read -p "Build OpenCV 3.1.0 for C++ and Octave (y/n): " yn
case $yn in
	[YyTt]* ) cd build
		sudo make uninstall
		cd ..
		rm -rf build
		mkdir build
		cd build

		# http://docs.opencv.org/3.1.0/d7/d9f/tutorial_linux_install.html
#		-D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.1.0/modules \
#		-D BUILD_opencv_aruco=OFF \
#		-D BUILD_opencv_bioinspired=OFF \
#		-D BUILD_opencv_ccalib=OFF \
#		-D BUILD_opencv_cnn_3dobj=OFF \
#		-D BUILD_opencv_contrib_world=OFF \
#		-D BUILD_opencv_dnn=OFF \
#		-D BUILD_opencv_dnns_easily_fooled=OFF \
#		-D BUILD_opencv_dpm=OFF \
#		-D BUILD_opencv_face=OFF \
#		-D BUILD_opencv_fuzzy=OFF \
#		-D BUILD_opencv_freetype=OFF \
#		-D BUILD_opencv_hdf=OFF \
#		-D BUILD_opencv_structured_light=OFF \
#		-D BUILD_opencv_text=OFF \

		cmake \
		-D CMAKE_BUILD_TYPE=Release -D BUILD_SHARED_LIBS=ON \
		-D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF -D BUILD_opencv_apps=OFF \
		-D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF \
		-D CMAKE_INSTALL_PREFIX=/usr/local \
		-D WITH_TBB=ON -D WITH_OPENMP=ON -D WITH_IPP=OFF \
		-D WITH_NVCUVID=OFF -D WITH_CUDA=OFF \
		-D WITH_OPENCL=ON \
		-D BUILD_opencv_python2=OFF \
		-D BUILD_opencv_python3=OFF \
		-D BUILD_opencv_java=OFF \
		-D ENABLE_PRECOMPILED_HEADERS=ON \
		../opencv-3.1.0 | tee cmake.log
	
		if [ -z "$TRAVIS" ]; then
			make #no multiple threads because it consumes too much ram
		else
			make -j4
		fi
		sudo make install
		sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv3.conf'
		sudo ldconfig
		cd ..
		;;
esac

# Octave MEX files
read -p "Build MEX files for Octave (y/n): " yn
case $yn in
	[YyTt]* )
		if [ ! -d "mexopencv-3.1.0" ]; then
			curl -L https://github.com/kyamagu/mexopencv/archive/v3.1.0.tar.gz > mexopencv.tar.gz
			tar -xf mexopencv.tar.gz
			rm -f mexopencv.tar.gz
		fi
		cd mexopencv-3.1.0
		make clean WITH_OCTAVE=true
		#make all contrib WITH_OCTAVE=true WITH_CONTRIB=true NO_CV_PKGCONFIG_HACK=true
		make all WITH_OCTAVE=true WITH_CONTRIB=false
		cd ..

		cat << EOF > ../octave/setup_opencv.m
#!/usr/local/bin/octave -f

addpath('${PWD}/mexopencv-3.1.0');
addpath('${PWD}/mexopencv-3.1.0/+cv/private');
disp('https://github.com/kyamagu/mexopencv/wiki/Gotchas');
EOF
		;;
esac

cd ..
