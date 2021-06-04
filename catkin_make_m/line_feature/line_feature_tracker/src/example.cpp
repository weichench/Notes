#include <iostream>
#include <chrono>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>

using namespace cv;
using namespace std;
using namespace cv::line_descriptor;
struct sort_descriptor_by_queryIdx
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ( a[0].queryIdx < b[0].queryIdx );
    }
};
struct sort_lines_by_response
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.response > b.response );
	//response表示了线特征的长度与图像最大尺寸的比值
    }
};
void ExtractLineSegment(const Mat &img, const Mat &image2, vector<KeyLine> &keylines,vector<KeyLine> &keylines2);
int main(int argc, char**argv)
{
    if(argc != 3)
        {
            cerr << endl << "Usage: ./Line path_to_image1 path_to_image2" << endl;
            return 1;
        }
    string imagePath1=string(argv[1]);
    string imagePath2=string(argv[2]);
    
cout<<"import two images"<<endl;
    Mat image1=imread(imagePath1);
    Mat image2=imread(imagePath2);
    imshow("ima1",image1);
    imshow("ima2",image2);
    waitKey(0);
    if(image1.data==NULL)
    {
        cout<<"the path is wrong"<<endl;
    }

    vector<KeyLine> keylines,keylines2;


    ExtractLineSegment(image1,image2,keylines,keylines2);

        return 0;

}
void ExtractLineSegment(const Mat &img, const Mat &image2, vector<KeyLine> &keylines,vector<KeyLine> &keylines2)
{
    Mat mLdesc,mLdesc2;

    vector<vector<DMatch>> lmatches;

    Ptr<BinaryDescriptor> lbd = BinaryDescriptor::createBinaryDescriptor();
    Ptr<line_descriptor::LSDDetector> lsd = line_descriptor::LSDDetector::createLSDDetector();

    cout<<"extract lsd line segments"<<endl;
    lsd->detect(img, keylines, 1.2,1);  
    lsd->detect(image2,keylines2,1.2,1);
    int lsdNFeatures = 50;
    cout<<"filter lines"<<endl;
    if(keylines.size()>lsdNFeatures)
    {
        sort(keylines.begin(), keylines.end(), sort_lines_by_response());
	//sort函数，默认升序排序，最后一个参数为比较函数，可实现降序，根据长度降序排序
	
        keylines.resize(lsdNFeatures);
        for( int i=0; i<lsdNFeatures; i++)
            keylines[i].class_id = i;
    }
    if(keylines2.size()>lsdNFeatures)
    {
        sort(keylines2.begin(), keylines2.end(), sort_lines_by_response());
        keylines2.resize(lsdNFeatures);
        for(int i=0; i<lsdNFeatures; i++)
            keylines2[i].class_id = i;
    }
    cout<<"lbd describle"<<endl;
    lbd->compute(img, keylines, mLdesc);
    lbd->compute(image2,keylines2,mLdesc2);//计算特征线段的描述子
    
    BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
    bfm->knnMatch(mLdesc, mLdesc2, lmatches, 2);
    vector<DMatch> matches;
    for(size_t i=0;i<lmatches.size();i++)
    {
        const DMatch& bestMatch = lmatches[i][0];
        const DMatch& betterMatch = lmatches[i][1];
        float  distanceRatio = bestMatch.distance / betterMatch.distance;
        if (distanceRatio < 0.75)
            matches.push_back(bestMatch);
    }

    cv::Mat outImg;
    std::vector<char> mask( lmatches.size(), 1 );
    drawLineMatches( img, keylines, image2, keylines2, matches, outImg, Scalar::all( -1 ), Scalar::all( -1 ), mask,
                     DrawLinesMatchesFlags::DEFAULT );

    imshow( "Matches", outImg );
    waitKey();

}


// cam2world(distorted2undistorted)
// 先经过内参变换，再进行畸变矫正
{
  cv::Point2f uv(u,v), px;
  const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
  cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
  cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
  xyz[0] = px.x;
  xyz[1] = px.y;
  xyz[2] = 1.0;
}
// 先经过镜头发生畸变，再成像过程，乘以内参
// world2cam(undistorted2distorted)
{
  double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
  x = uv[0];
  y = uv[1];
  r2 = x*x + y*y;
  r4 = r2*r2;
  r6 = r4*r2;
  a1 = 2*x*y;
  a2 = r2 + 2*x*x;
  a3 = r2 + 2*y*y;
  cdist = 1 + d_[0]*r2 + d_[1]*r4 + d_[4]*r6;
  xd = x*cdist + d_[2]*a1 + d_[3]*a2;
  yd = y*cdist + d_[2]*a3 + d_[3]*a1;
  px[0] = xd*fx_ + cx_;
  px[1] = yd*fy_ + cy_;
}




// cam2world(distorted2undistorted)
// 先经过内参变换，再进行畸变矫正
{
  cv::Point2f uv(u,v), px;
  const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
  cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
  cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
  xyz[0] = px.x;
  xyz[1] = px.y;
  xyz[2] = 1.0;
}
// 先经过镜头发生畸变，再成像过程，乘以内参
// world2cam(undistorted2distorted)
{
  double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
  x = uv[0];
  y = uv[1];
  r2 = x*x + y*y;
  r4 = r2*r2;
  r6 = r4*r2;
  a1 = 2*x*y;
  a2 = r2 + 2*x*x;
  a3 = r2 + 2*y*y;
  cdist = 1 + d_[0]*r2 + d_[1]*r4 + d_[4]*r6;
  xd = x*cdist + d_[2]*a1 + d_[3]*a2;
  yd = y*cdist + d_[2]*a3 + d_[3]*a1;
  px[0] = xd*fx_ + cx_;
  px[1] = yd*fy_ + cy_;
}