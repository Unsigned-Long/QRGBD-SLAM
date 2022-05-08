cd ORB_SLAM3/Thirdparty/
sudo rm -rf DBoW2/build/* Sophus/build/* g2o/build/* DBoW2/lib/* g2o/lib/*
cd -
cd darknet/build/
sudo rm -rf *
cmake ..
make
cd -
sudo rm -r build-qrgbd-Desktop_Qt_6_1_3_GCC_64bit-Release
cp darknet/build/libdarknet.so  yolov4-learn/lib/
cd ORB_SLAM3
sudo rm -r build
./build.sh
