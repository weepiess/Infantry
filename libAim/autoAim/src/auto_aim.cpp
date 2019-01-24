#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>

AutoAim::AutoAim(){}

AutoAim::AutoAim(int width, int height){
    IMG_WIDTH = width;
    IMG_HEIGHT = height;
    resetROI();
    resizeCount = 0;

    //初始化三维坐标点
    pnpSolver.pushPoints3D(-130, -60, 0);
    pnpSolver.pushPoints3D(130, -60, 0);
    pnpSolver.pushPoints3D(130, 60, 0);
    pnpSolver.pushPoints3D(-130, 60, 0);
    //初始化相机参数
    //pnpSolver.setCameraMatrix(1020.80666, 0., 695.74256, 0.,1020.80666,388.82902, 0., 0., 1.);
    pnpSolver.setCameraMatrix(1044.11801, 0., 637.0385, 0.,1046.6575,467.3094, 0., 0., 1.);
    //pnpSolver.setDistortionCoef(0.0058917, 0.269857, 0.0026559, 0.00903601,0.393959);
    pnpSolver.setDistortionCoef(-0.1018, 0.1015, -0.0135, -0.00073262,0.000241165);
    aim_predict.model_init();
    bestCenter.x=-1;
}


AutoAim::~AutoAim(){}

Point2d cal_x_y(RotatedRect &rect, int is_up){
    float angle = (90-rect.angle)*CV_PI/180;
    Point2d point;
    if(is_up){
        point.x = rect.center.x + rect.size.height/2*cos(angle);
        point.y = rect.center.y - rect.size.height/2*sin(angle);
    } else {
        point.x = rect.center.x - rect.size.height/2*cos(angle);
        point.y = rect.center.y + rect.size.height/2*sin(angle);
    }
    return point;
}

void AutoAim::resetROI(){
    rectROI.x = 0;
    rectROI.y = 0;
    rectROI.width = IMG_WIDTH;
    rectROI.height = IMG_HEIGHT;
}

void AutoAim::set_parameters(int angle,int inside_angle, int height, int width){
    param_diff_angle = angle;
    param_inside_angle = inside_angle;
    param_diff_height = height;
    param_diff_width = width;
}

//图像预处理
bool AutoAim::setImage(Mat &img){
    if(img.empty()) return false;
    Mat channel[3], Mask, diff;
    int thresh = 40, substract_thresh = 100;
    resetROI();
    mask = img(rectROI);
    split(mask, channel);
    Mask = channel[0];
    diff = channel[0] - channel[1];
    GaussianBlur(Mask, Mask, Size(5,5), 0);
    threshold(Mask, Mask, thresh, 255, THRESH_BINARY);
    threshold(diff, diff, substract_thresh, 255, THRESH_BINARY);
    Mat element = getStructuringElement( MORPH_ELLIPSE, Size(1, 3));
    for (int i = 0; i < 8; ++i){
        dilate( diff, diff, element);
    }   
    bitwise_and(Mask, diff, mask);
    
    /*
    if(enemyColor == color_blue){
        threshold(channel[0] - channel[2], mask, 0, 255, THRESH_BINARY+THRESH_OTSU);
    } else if (enemyColor == color_red){
        threshold(channel[2] - channel[0], mask, 75, 255, THRESH_BINARY);
    } else {
        cout<<"enemyColor has an improper value, please check it again!!!";
        return false;
    }
    */
    //Canny(mask, mask, 3, 9, 3);
    //imshow("mask", mask);
    return true;
}

//寻找灯管
void AutoAim::findLamp_rect(vector<RotatedRect> &pre_armor_lamps){
    pre_armor_lamps.clear();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    findContours(mask, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    RotatedRect temp;
    float lastCenterX = 0, lastCenterY = 0;
    if(contours.size()<40){
        for(int i=0;i<contours.size();i++){
            if(contours[i].size()>5){
                temp = adjustRRect(minAreaRect(contours[i]));//寻找最小外接矩形
                if(abs(temp.angle)>45) continue;//旋转矩形角度小于45度，则忽略
                pre_armor_lamps.push_back(temp);
            }
        }
    }
}
void AutoAim::match_lamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps){
    cv::FileStorage f;
    f = FileStorage("../libAim/res/aimdata.yaml",0); //READ

    //权重
    float yx_ratio;
    float params_max_height_ratio, params_max_dis_height_ratio, params_min_dis_height_ratio, params_max_allow_angle, params_max_yx_diff_ratio;
    int height_diff_weight,angle_diff_weight,height_ratio_weight,yx_ratio_weight,ratio_max,ratio_min;
   /* f["height_diff_weight"] >> height_diff_weight;
    f["angle_diff_weight"] >> angle_diff_weight;
    f["height_ratio_weight"] >> height_ratio_weight;
    f["yx_ratio_weight"] >> yx_ratio_weight;
    f["ratio_max"] >> ratio_max;
    f["ratio_min"] >> ratio_min;
    f["params_max_height_ratio"] >> params_max_height_ratio;
    f["params_max_dis_height_ratio"] >> params_max_dis_height_ratio;
    f["params_min_dis_height_ratio"] >> params_min_dis_height_ratio;
    f["params_max_yx_diff_ratio"] >> params_max_yx_diff_ratio;
    f["params_max_allow_angle"] >> params_max_allow_angle;
    f.release();*/
    height_diff_weight=2;
    angle_diff_weight=5;
    height_ratio_weight=2;
    yx_ratio_weight=3;
    ratio_max=6;
    ratio_min= 1;
    params_max_height_ratio= 2;
    params_max_dis_height_ratio= 10;
    params_min_dis_height_ratio= 1;
    params_max_yx_diff_ratio= 1;
    params_max_allow_angle=45;
    // int angle_diff_weight = 3;
    // int height_diff_weight = 2;
    // int angle_diff_weight = 5;
    // int height_ratio_weight = 2;
    // int yx_ratio_weight = 3;
    //初始化
    int size = pre_armor_lamps.size();
    vector<float> diff(size,0x3f3f3f3f);
    vector<float> best_match_index(size,-1);
    //#pragma omp parallel for
    // for(int i=0; i<size; i++){
    //     diff[i] = 0x3f3f3f3f;
    //     best_match_index[i] = -1;
    // }
    for(int i=0; i<pre_armor_lamps.size(); i++){
        cout<<pre_armor_lamps.at(i).angle<<" "<<pre_armor_lamps.at(i).size.height<<" "<<pre_armor_lamps.at(i).size.width<<" ---- ";
    }
    cout<<endl;
    //计算灯管匹配之间的花费
    float height_ratio,dist, avg_height, diff_angle, diff_height, ratio, totalDiff,inside_angle,diff_width,dis_height_ratio;
    int i,j;
    //#pragma omp parallel for
    for(i=0; i<size; i++){
        float currDiff = 0x3f3f3f3f;
        int currIndex = -1;
        const RotatedRect &current = pre_armor_lamps[i];
        int theta_current = current.angle;
        for(j=i+1;j<size; j++){
            //计算比例，筛选灯管
            const RotatedRect &compare = pre_armor_lamps[j];
            int theta_compare = compare.angle;
        
            //灯条角度差超过设定角度忽略
            diff_angle = abs(theta_compare - theta_current);
            if(diff_angle > params_max_allow_angle) continue;

            //y差值与x差值超过设定值忽略
            if(current.center.x - compare.center.x == 0) yx_ratio=100;
            else yx_ratio = abs(current.center.y-compare.center.y)/abs(current.center.x-compare.center.x);
            if(yx_ratio > params_max_yx_diff_ratio) continue;
            //内角小于设定角度忽略

            //两灯条高度比例不在范围内则忽略
             if(compare.size.height > current.size.height)
                height_ratio = compare.size.height*1.0f/current.size.height;
            else
                height_ratio = current.size.height*1.0f/compare.size.height;
            //LOG_INFO<<height_ratio;
            if(height_ratio > params_max_height_ratio) continue;
            //LOG_INFO<<"height_ratio";

            //灯条之间的距离与灯条的平均长度之比需要在一定范围之内
            dist = ImageTool::calc2PointApproDistance(compare.center, current.center);
            avg_height = (compare.size.height + current.size.height) / 2;
            dis_height_ratio = dist / avg_height;
            //LOG_INFO<<dis_height_ratio;
            if(dis_height_ratio > params_max_dis_height_ratio || dis_height_ratio < params_min_dis_height_ratio) continue;
            //角度差约束会在几度之内，高度比例约束会在1到1.x之内，内角约束大致在几十度到90度(目前70-90)，
            //实际上用90度减去角度应该在0-几十度之内，且越小越好
            //用归一化的值算最后的花费，避免不同值的取值范围不同
            totalDiff = angle_diff_weight * (diff_angle/params_max_allow_angle) //角度花费
                        + height_ratio_weight * ((height_ratio-1)/(params_max_height_ratio-1)) //高度比例花费
                        + yx_ratio_weight * (yx_ratio/params_max_yx_diff_ratio); //内角花费

              //更新j代表的灯条的最小花费
            if(diff.at(j) > totalDiff)
                diff.at(j) = totalDiff;

            //更新i代表的当前灯条的最优匹配
            if(diff.at(i) > totalDiff){
                diff.at(i) = totalDiff;
                currIndex = j;
            }            
        }
        if(currIndex==-1) continue;
        else {
            best_match_index.at(i) = currIndex;
            best_match_index.at(currIndex) = i;
        }
    }
    //#pragma omp parallel for
    for(i=0; i<size; i++){
        //cout<<best_match_index[i]<<endl;
        int index = best_match_index[i];
        if(index == -1 || index <= i) continue;
        if(i == best_match_index[index]){
            real_armor_lamps.push_back(pre_armor_lamps[i]);
            real_armor_lamps.push_back(pre_armor_lamps[index]);
        }
    }
}
void AutoAim::select_armor(vector<RotatedRect> real_armor_lamps){
    int lowerY=0;
    int lowerIndex=-1;
    int hero_index=-1;
    bestCenter.x=-1;
    //最优装甲板逻辑
    for(int i=0; i<real_armor_lamps.size(); i+=2){
        if(i+1 >= real_armor_lamps.size()) break;
        int y = (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2;
        int x = abs(real_armor_lamps[i].center.x-real_armor_lamps[i+1].center.x);
        if(x/real_armor_lamps[i].size.height>4){  
            hero_index=i; 
            pnpSolver.clearPoints3D();
            pnpSolver.pushPoints3D(-115, -47, 0);
            pnpSolver.pushPoints3D(115, -47, 0);
            pnpSolver.pushPoints3D(115, 47, 0);
            pnpSolver.pushPoints3D(-115, 47, 0);
            break;
        }
        if(y > lowerY){
            lowerY = y;
            lowerIndex = i;
        }
    }
    if(hero_index!=-1){
        resizeCount=0;
        count++;
        if(real_armor_lamps[hero_index].center.x > real_armor_lamps[hero_index+1].center.x){
            swap(real_armor_lamps[hero_index],real_armor_lamps[hero_index+1]);//确保偶数为左灯条，奇数为右灯条
        }
        int height = (real_armor_lamps[hero_index].size.height + real_armor_lamps[hero_index+1].size.height)/2;
        //当灯条高度小于10个像素点时放弃锁定，重新寻找合适目标
        if(height > 10){
            //cout<<rectROI.x<<" "<<rectROI.y<<endl;
            bestCenter.x = (real_armor_lamps[hero_index].center.x + real_armor_lamps[hero_index+1].center.x)/2 + rectROI.x;
            bestCenter.y = (real_armor_lamps[hero_index].center.y + real_armor_lamps[hero_index+1].center.y)/2 + rectROI.y;
        }else{
            resetROI();
            count=0;
        }
    }
    //优先锁定图像下方装甲板
    else if(lowerIndex == -1){
        resizeCount++;
        count=0;
        if(!broadenRect(rectROI) || resizeCount>3){
            resetROI();
            resizeCount = 0;
        }
    } 
    else if(lowerIndex != -1) {
        resizeCount = 0; 
        count++;
        //cout<<real_armor_lamps[lowerIndex].x<<"  "<<real_armor_lamps[lowerIndex+1].x<<endl;
	    if(real_armor_lamps[lowerIndex].center.x > real_armor_lamps[lowerIndex+1].center.x){
            swap(real_armor_lamps[lowerIndex],real_armor_lamps[lowerIndex+1]);//确保偶数为左灯条，奇数为右灯条
        }
        int height = (real_armor_lamps[lowerIndex].size.height + real_armor_lamps[lowerIndex+1].size.height)/2;
        //当灯条高度小于10个像素点时放弃锁定，重新寻找合适目标
        if(height > 10){
            //cout<<rectROI.x<<" "<<rectROI.y<<endl;
            bestCenter.x = (real_armor_lamps[lowerIndex].center.x + real_armor_lamps[lowerIndex+1].center.x)/2 + rectROI.x;
            bestCenter.y = (real_armor_lamps[lowerIndex].center.y + real_armor_lamps[lowerIndex+1].center.y)/2 + rectROI.y;
            //cout<<bestCenter<<endl;
        } else{
		    resetROI();
            count=0;
    	}
    }

    if(bestCenter.x!=-1){
        clock_t finish = clock();
        best_lamps[0] = real_armor_lamps[lowerIndex];
        best_lamps[1] = real_armor_lamps[lowerIndex+1];
        best_lamps[0].center.x+=rectROI.x;
        best_lamps[0].center.y+=rectROI.y;
        best_lamps[1].center.x+=rectROI.x;
        best_lamps[1].center.y+=rectROI.y;
        rectROI.x = (best_lamps[0].center.x + best_lamps[1].center.x)/2 - (best_lamps[1].center.x - best_lamps[0].center.x);
        rectROI.y = (best_lamps[0].center.y + best_lamps[1].center.y)/2 - (best_lamps[0].size.height + best_lamps[1].size.height)/2;
        rectROI.height = best_lamps[0].size.height + best_lamps[1].size.height;
        rectROI.width = 2*(best_lamps[1].center.x - best_lamps[0].center.x);
        cout<<rectROI.x<<endl;
        if(!makeRectSafe(rectROI)){
            resetROI();
	    }
    }
}

BaseAim::AimResult AutoAim::aim(Mat &src, float currPitch, float currYaw, Point2f &pitYaw,int is_predict,bool &if_shoot,float time_delay){

    
     if(bestCenter.x != -1){
         circle(src, bestCenter, 20, Scalar(255,255,255), 5);
         rectangle(src, rectROI, Scalar(255,0,0), 2);
     }
//    imshow("src",src);
  //  waitKey(1);
    if(bestCenter.x!=-1){
        //circle(src, Point(xc1,yc1), 20, Scalar(255,255,0), 2);
        //circle(src, Point(xc2,yc2), 20, Scalar(255,255,0), 2);
        count++;
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[0],1));//P1
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[1],1));//P3
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[1],0));//P2
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[0],0));//P4

        pnpSolver.solvePnP();
        pnpSolver.clearPoints2D();
        //pnpSolver.showParams();

        Point3d tvec = pnpSolver.getTvec();
        cout<<"x: "<<tvec.x<<"y: "<<tvec.y<<"z: "<<tvec.z<<endl;
        //vector<Point3f> worldpoint;
        //worldpoint.push_back(Point3f(tvec.x,tvec.y-37,tvec.z));
        //vector<Point2f>framepoint = pnpSolver.WorldFrame2ImageFrame(worldpoint);
        // armor.x = (best_lamps[0].center.x + best_lamps[1].center.x)/2 - (best_lamps[1].center.x - best_lamps[0].center.x)/4;
        // armor.y = (best_lamps[0].center.y + best_lamps[1].center.y)/2  - (best_lamps[0].size.height + best_lamps[1].size.height) ;
        // armor.height = (best_lamps[0].size.height + best_lamps[1].size.height);
        // armor.width = 1*(best_lamps[1].center.x - best_lamps[0].center.x);
        // if(armor.y<0){
        //     return AIM_TARGET_NOT_FOUND;
        // }

        if(is_predict){
	    pitYaw = calPitchAndYaw(tvec.x,tvec.y, tvec.z, tvec.z/45, 20, 170, currPitch, currYaw);
	    cout<<"init yaw "<<currYaw<<endl;
	    measurement.at<float>(0) = currYaw + pitYaw.y;
	    cout<<"Measurement: "<<measurement<<endl;            
            if(count==1){
                Mat statePost=(Mat_<float>(2, 1) << currYaw+pitYaw.y,0);
                aim_predict.model_init();
                aim_predict.reset_kf_statepost(statePost);
            }
	   // measurement.at<float>(0) = currYaw + pitYaw.y;  
            Mat Predict = this->aim_predict.predict(measurement,time_delay);
           // pitYaw = calPitchAndYaw(tvec.x,tvec.y, tvec.z, tvec.z/17, -100, 170, currPitch, currYaw);
	    cout<<"predict: "<<Predict<<endl;
            float predict_angle=Predict.at<float>(1)*(2*time_delay+tvec.z/27);
	    cout<<"time delay "<<time_delay<<endl;
	    cout<<"add angle: "<<predict_angle<<endl;
            if_shoot=aim_predict.shoot_logic(pitYaw.y,Predict.at<float>(1),predict_angle);
            pitYaw.y += predict_angle;
            
        }else{   
            pitYaw = calPitchAndYaw(tvec.x, tvec.y, tvec.z, tvec.z/45, -50, 170, currPitch, currYaw);
        }
        return AIM_TARGET_FOUND;
    }
    return AIM_TARGET_NOT_FOUND;
}





