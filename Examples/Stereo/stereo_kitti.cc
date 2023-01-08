/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
// 스트림을 이용한 입출력 기능 제공
#include<algorithm>
// 원소들에 대해 작업할 수 있는 함수 정의
#include<fstream>
// 파일 입출력 관련 함수 정의
#include<iomanip>
// setw 와 같은 함수 정의
#include<chrono>
// 시간 관련 라이브러리
#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
// 실질적으로 첫번째 param만 입력값, 나머지는 출력 값에 해당

int main(int argc, char **argv)
// argc : argument 갯수, argv : argument 값
{
    if(argc != 4)
    // argument 갯수가 4개가 아닐 경우
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    // 좌안 이미지 path
    vector<string> vstrImageLeft;
    // 우안 이미지 path
    vector<string> vstrImageRight;
    // 이미지의 timestamp(파일이름)
    vector<double> vTimestamps;

    LoadImages(string(argv[3]) // 이미지 sequence 경로
    , vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();
    /**
     * 이미지 갯수 == path내 존재하는 이미지 갯수
     * ! vstrImageLeft에 데이터 입력은 언제? => vector를 &로 불러와 LoadImages함수 내에서 입력.
     */

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
    /**
    * * System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);
    * @param argv1 => Voc 파일(txt) path
    * @param argv2 => Setting 파일(yaml) path
    * @param ORB_SLAM2::System::STEREO => 어떤 센서를 사용할 것인지?
    * @param true => viewer를 킬지 말지?
    */

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    vTimesTrack.resize(nImages);
    /**
     * ! vTimesTrack 벡터를 nImages의 크기로 미리 조절
     * ? 왜 필요한가?
     * * 동적 배열에 미리 공간을 할당하여 연산 복잡도를 줄인다
     * * reserve => 공간 확보만 // resize => 초기화 까지
     */

    cout<<endl;
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    /**
     * ! 행렬을 표현하기 위한 자료형. n차원의 단일 채널 또는 멀티 채널 배열 표현가능
     */
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    // 입력 버퍼 스트림 선언
    ifstream fTimes;
    // sequence 파일 경로
    string strPathTimeFile = strPathToSequence + "/times.txt";
    // open은 char*를 매개변수로 받기 때문에, c_str를 통해 string을 char*로 변환해준다.
    fTimes.open(strPathTimeFile.c_str());
    
    // fTimes가 끝날때 까지
    while(!fTimes.eof())
    {
        string s;
        // fTimes내 '\n'을 만날때 까지의 한 줄을 s에 담는다.
        getline(fTimes,s);
        // s내 값이 존재하면 vTimestamps에 해당 값을 담는다
        if(!s.empty())
        {
            /**
             * ! stringstream : 주어진 문자열에서 필요한 자료형에 맞는 정보를 꺼낼 때 유용하게 사용
             * ? 왜 사용?
             * Todo
             * * e+0x 형태로 입력된 string을 소숫점 형태로 변환해서 제공함
             */
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            // timestamp의 double 형 기록
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        // 000000의 형태로 파일들이 index 되도록 설정
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
