#!/usr/bin/env bash

function install_opencv {
	# OpenCV dependencies
	sudo apt-get install -y build-essential
	sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
	sudo apt-get install -y python-dev python-numpy python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

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

	pkg-config opencv
	if [ "$?" -ne 0 ]; then
		cd build
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
	
		make -j4
		sudo make install
		sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv3.conf'
		sudo ldconfig
		cd ..
	fi
}

case "$BUILD" in
	tex)
		yes "" | sudo add-apt-repository ppa:texlive-backports/ppa
		sudo apt-get update -qq
		sudo apt-get install -y texlive texlive-base texlive-latex-recommended texlive-latex-extra texlive-binaries texlive-science texlive-lang-polish texlive-bibtex-extra texlive-fonts-recommended
		;;

	library)
		install_opencv
		;;
		
	android)
		install_opencv
		;;
esac


