#include <gtest/gtest.h>
#include <uav_vision/h_stroke_detector.h>
#include <uav_vision/landing_h_mask.h>

cv::Mat hMask() {
  cv::Mat m=cv::Mat::zeros(400,600,CV_8UC1);
  cv::rectangle(m,{140,45},{210,355},255,-1);
  cv::rectangle(m,{390,45},{460,355},255,-1);
  cv::rectangle(m,{140,165},{460,235},255,-1);
  return m;
}

TEST(HStroke, UsesInnerEdgesWhenOuterStrokeEndsAreCropped) {
  auto mask=hMask()(cv::Rect(0,100,600,280)).clone();
  uav_vision::HStrokeObservation h;
  ASSERT_TRUE(uav_vision::detectHStrokes(mask,24,h));
  EXPECT_NEAR(h.center.x,300,3);
  EXPECT_NEAR(h.center.y,100,3);
}

TEST(HStroke, WorksWithRotatedH) {
  cv::Mat rotated;cv::rotate(hMask(),rotated,cv::ROTATE_90_CLOCKWISE);
  uav_vision::HStrokeObservation h;
  ASSERT_TRUE(uav_vision::detectHStrokes(rotated,24,h));
  EXPECT_NEAR(h.center.x,199,3);
  EXPECT_NEAR(h.center.y,300,3);
}

TEST(HStroke, RejectsCircleCrossAndU) {
  for(int kind=0;kind<3;++kind){
    cv::Mat mask=cv::Mat::zeros(400,600,CV_8UC1);
    if(kind==0)cv::circle(mask,{300,200},150,255,25);
    if(kind==1){cv::rectangle(mask,{270,40},{330,360},255,-1);cv::rectangle(mask,{140,170},{460,230},255,-1);}
    if(kind==2){cv::rectangle(mask,{140,45},{210,355},255,-1);cv::rectangle(mask,{390,45},{460,355},255,-1);cv::rectangle(mask,{140,285},{460,355},255,-1);}
    uav_vision::HStrokeObservation h;
    EXPECT_FALSE(uav_vision::detectHStrokes(mask,24,h));
  }
}

int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

TEST(HSegmentation, TintedInkRemainsHWithExposureChanges) {
  const auto silhouette = hMask();
  for (const auto& ink : std::vector<cv::Scalar>{{25,20,55},{15,45,20},{50,20,15},{30,30,30}}) {
    cv::Mat bgr(silhouette.size(),CV_8UC3,cv::Scalar(200,200,200));
    bgr.setTo(ink,silhouette);
    for (double gain : {0.45,0.75,1.0,1.2}) {
      cv::Mat exposed;bgr.convertTo(exposed,-1,gain,5.0);
      const auto mask=uav_vision::landingHMask(exposed,"grayscale_otsu",90,110,15.0);
      uav_vision::HStrokeObservation h;
      ASSERT_TRUE(uav_vision::detectHStrokes(mask,24,h));
      EXPECT_NEAR(h.center.x,300,3);EXPECT_NEAR(h.center.y,200,3);
    }
  }
}

TEST(HSegmentation, UniformAndInsufficientContrastAreNotEvidence) {
  for (int level : {0,50,128,255}) {
    cv::Mat im(200,200,CV_8UC3,cv::Scalar::all(level));
    EXPECT_EQ(0,cv::countNonZero(uav_vision::landingHMask(im,"grayscale_otsu",90,110,15)));
  }
  cv::Mat low(400,600,CV_8UC3,cv::Scalar::all(100));low.setTo(cv::Scalar::all(95),hMask());
  EXPECT_EQ(0,cv::countNonZero(uav_vision::landingHMask(low,"grayscale_otsu",90,110,15)));
}

TEST(HSegmentation, LegacyColorGateIsOptionalNotMorphologyIdentity) {
  cv::Mat im(400,600,CV_8UC3,cv::Scalar::all(200));im.setTo(cv::Scalar(25,20,55),hMask());
  const auto legacy=uav_vision::landingHMask(im,"legacy_hsv",90,110,15);
  EXPECT_EQ(0,cv::countNonZero(legacy));
  EXPECT_THROW(uav_vision::landingHMask(im,"typo",90,110,15),std::invalid_argument);
}
