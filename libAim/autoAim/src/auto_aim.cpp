#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>


AutoAim::AutoAim(){}


AutoAim::~AutoAim(){}

void AutoAim::init(int width, int height,Aim_assistant* checker){
    ratio1_max = -1;
    ratio1_min = 1000;
    ratio2_max = -1;
    ratio2_min = 1000;
    IMG_WIDTH = width;
    IMG_HEIGHT = height;
    resetROI();
    resizeCount = 0;

    //初始化三维坐标点(小装甲)
    pnpSolver.clearPoints3D();
    pnpSolver.pushPoints3D(-65, -33, 0);
    pnpSolver.pushPoints3D(65,  -33, 0);
    pnpSolver.pushPoints3D(65, 33, 0);
    pnpSolver.pushPoints3D(-65, 33, 0);

    //初始化相机参数
    pnpSolver.setCameraMatrix(1044.11801, 0., 637.0385, 0.,1046.6575,467.3094, 0., 0., 1.);
    pnpSolver.setDistortionCoef(-0.1018, 0.1015, -0.0135, -0.00073262,0.000241165);
    aim_predict.model_init();
    bestCenter.x=-1;
    id_checker = checker;
}
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
    key = 0;
}

//图像预处理
bool AutoAim::setImage(Mat &img){
    if(img.empty()) return false;
    img.copyTo(image);
    Mat channel[3], Mask, diff;
    int thresh = 40, substract_thresh = 110;
    rectangle(img,rectROI,Scalar(255,255,0));
    resetROI();
    img = img(rectROI);
    medianBlur(img,img,1);
    split(img, channel);
    mask = channel[0] - channel[2];
    threshold(mask, mask, substract_thresh, 255, THRESH_BINARY);
    // imshow("mask",mask);
    // waitKey(1);
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
                for(int j=0; j<contours[i].size(); j++)
                circle(image,contours[i][j],2,Scalar(0,255,0),2);
                temp = adjustRRect(minAreaRect(contours[i]));//寻找最小外接矩形
                if(abs(temp.angle)>45) continue;//旋转矩形角度小于45度，则忽略
                pre_armor_lamps.push_back(temp);
            }
        }
    }
}
void AutoAim::match_lamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps){
    
    //权重
    float yx_ratio;
    float params_max_height_ratio, params_max_dis_height_ratio, params_min_dis_height_ratio, params_max_allow_angle, params_max_yx_diff_ratio;
    int height_diff_weight,angle_diff_weight,height_ratio_weight,yx_ratio_weight,ratio_max,ratio_min;
    //权重
    angle_diff_weight=6;
    height_ratio_weight=2;
    yx_ratio_weight=3;

    ratio_max=6;
    ratio_min= 1;
    params_max_height_ratio= 1.5;
    params_max_dis_height_ratio= 4;
    params_max_yx_diff_ratio= 1; //一对灯条最大侧向旋转角
    params_max_allow_angle=2; //一对灯条最大允许角度差
    int size = pre_armor_lamps.size();
    vector<float> diff(size,0x3f3f3f3f);
    vector<float> best_match_index(size,-1);
    //计算灯管匹配之间的花费
    float height_ratio,dist, avg_height, diff_angle, diff_height, ratio, totalDiff,inside_angle,diff_width,dis_height_ratio;
    int i,j;

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
            
            diff_angle = fabsf(theta_compare - theta_current);
            if(diff_angle > params_max_allow_angle) continue;
            if(diff_angle > max)
                max = diff_angle;

            //y差值与x差值超过设定值忽略
            if(current.center.x - compare.center.x == 0) yx_ratio=100;
            else yx_ratio = fabsf(current.center.y-compare.center.y)/fabsf(current.center.x-compare.center.x);
            if(yx_ratio > params_max_yx_diff_ratio) continue;
            //内角小于设定角度忽略

            //两灯条高度比例不在范围内则忽略
             if(compare.size.height > current.size.height)
                height_ratio = compare.size.height*1.0f/current.size.height;
            else
                height_ratio = current.size.height*1.0f/compare.size.height;

            if(height_ratio > params_max_height_ratio) continue;


            //灯条之间的距离与灯条的平均长度之比需要在一定范围之内
            dist = ImageTool::calc2PointDistance(compare.center, current.center);
            avg_height = (compare.size.height + current.size.height) / 2.0;
            dis_height_ratio = dist / avg_height;

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

    for(i=0; i<size; i++){
        int index = best_match_index[i];
        if(index == -1 || index <= i) continue;
        if(i == best_match_index[index]){
            real_armor_lamps.push_back(pre_armor_lamps[i]);
            real_armor_lamps.push_back(pre_armor_lamps[index]);
        }
    }
}
void AutoAim::select_armor(vector<RotatedRect> real_armor_lamps){
    int lowerY=100;
    int lowerIndex=-1;
    int hero_index=-1;
    bestCenter.x=-1;
    vector<int> armor_detected;
    //最优装甲板逻辑
    for(int i=0; i<real_armor_lamps.size(); i+=2){
        if(i+1 >= real_armor_lamps.size()) break;
        Rect armor_area;
        if(real_armor_lamps[i].center.x > real_armor_lamps[i+1].center.x){
            swap(real_armor_lamps[i],real_armor_lamps[i+1]);//确保偶数为左灯条，奇数为右灯条
        }
        Point2d left_up = cal_x_y(real_armor_lamps[i],1);
        Point2d right_up = cal_x_y(real_armor_lamps[i+1],1);
        armor_area.x = left_up.x;
        armor_area.y = left_up.y-0.2*real_armor_lamps[i].size.height;
        armor_area.height = (1.5)*real_armor_lamps[i].size.height;
        armor_area.width = abs(right_up.x - left_up.x);
        int number = id_checker->check_armor(source_image(armor_area));  
        armor_detected.push_back(number);
    }
    for(int i=0;i<armor_detected.size();i++){
        if(armor_detected[i] == 2){
            pnpSolver.clearPoints3D();
            pnpSolver.pushPoints3D(-109, -33, 0);
            pnpSolver.pushPoints3D(109,  -33, 0);
            pnpSolver.pushPoints3D(109, 33, 0);
            pnpSolver.pushPoints3D(-109, 33, 0);
	        cout<<"find hero !!!!!!"<<endl;
            hero_index = 2*i;
            lowerIndex = 2*i;
            break;
        }
        else if(armor_detected[i] != 5 && armor_detected[i]!=-1){
            float y = 720 - (real_armor_lamps[2*i].center.y + real_armor_lamps[2*i+1].center.y)/2;
            float x = fabs(640-(real_armor_lamps[2*i].center.x + real_armor_lamps[2*i+1].center.x)/2);
            y = y / 720;
            x = x / 640;
            float score = 0.6 * y + 0.4 * x;
            if(score > lowerY){
                lowerY = score;
                lowerIndex = 2 * i;
            }
        }
    }
    if(hero_index!=-1){
        resizeCount=0;
        count++;
        int height = (real_armor_lamps[hero_index].size.height + real_armor_lamps[hero_index+1].size.height)/2;
        //当灯条高度小于10个像素点时放弃锁定，重新寻找合适目标
        if(height > 1){
            
            bestCenter.x = (real_armor_lamps[hero_index].center.x 
                        + real_armor_lamps[hero_index+1].center.x)/2 + rectROI.x ;
            bestCenter.y = (real_armor_lamps[hero_index].center.y 
                        + real_armor_lamps[hero_index+1].center.y)/2 +rectROI.y;
        }else{
            resetROI();
            count=0;
        }
    }
    //优先锁定图像下方装甲板
    else {if(lowerIndex == -1){
        resizeCount++;
        count=0;
        if(!broadenRect(rectROI)){ //|| resizeCount>5){
            resetROI();
            resizeCount = 0;
        }
    } 
    else if(lowerIndex != -1) {
	cout<<" attacking Infantry!!!  :) "<<endl;
        pnpSolver.clearPoints3D();
        pnpSolver.pushPoints3D(-65, -33, 0);
        pnpSolver.pushPoints3D(65,  -33, 0);
        pnpSolver.pushPoints3D(65, 33, 0);
        pnpSolver.pushPoints3D(-65, 33, 0);
        resizeCount = 0; 
        count++;
        int height = (real_armor_lamps[lowerIndex].size.height + real_armor_lamps[lowerIndex+1].size.height)/2;
        //当灯条高度小于5个像素点时放弃锁定，重新寻找合适目标
        if(height > 5){
            bestCenter.x = (real_armor_lamps[lowerIndex].center.x + real_armor_lamps[lowerIndex+1].center.x)/2 + rectROI.x;
            bestCenter.y = (real_armor_lamps[lowerIndex].center.y + real_armor_lamps[lowerIndex+1].center.y)/2 + rectROI.y;
        } else{
		    resetROI();
            count=0;
    	}
    }}

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
	if(hero_index==-1) resetROI();
        //cout<<rectROI.x<<endl;
        if(!makeRectSafe(rectROI)){
            resetROI();
	    }
    }
}
BaseAim::AimResult AutoAim::aim(Mat &src, float currPitch, float currYaw, Point2f &pitYaw,int is_predict,bool &if_shoot,float time_delay){

    bool isKalman = true;
    if(bestCenter.x!=-1){
        count++;
        Mat frame;
        src.copyTo(frame);
        Rect rect;
        Point2d left_up = cal_x_y(best_lamps[0],1);
        Point2d right_up = cal_x_y(best_lamps[1],1);
        rect.x = left_up.x;
        rect.y = left_up.y-0.15*best_lamps[0].size.height;
        rect.height = (1.5)*best_lamps[0].size.height;
        rect.width = abs(right_up.x - left_up.x);
        rectangle(src,rect,Scalar(0,255,255));
        int number = id_checker->check_armor(frame(rect));
        if(number != -1) putText(src,to_string(number),Point(15,15),1,1,Scalar(0,0,255),2);
        pnpSolver.pushPoints2D(left_up);//P1
        pnpSolver.pushPoints2D(right_up);//P3
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[1],0));//P2
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[0],0));//P4
        imshow("src",src);
        waitKey(1);
        pnpSolver.solvePnP();
        pnpSolver.clearPoints2D();
        //pnpSolver.showParams();

        Point3d tvec = pnpSolver.getTvec();
        if(is_predict && isKalman){
	        pitYaw = calPitchAndYaw(tvec.x,tvec.y, tvec.z, tvec.z/63, -120, 170, currPitch, currYaw);
	        measurement.at<float>(0) = currYaw + pitYaw.y;           
            if(count==1||count==2){
                Mat statePost=(Mat_<float>(2, 1) << currYaw+pitYaw.y,0);
                aim_predict.model_init();
                aim_predict.reset_kf_statepost(statePost);
            } 
            Mat Predict = this->aim_predict.predict(measurement,time_delay);
            float predict_angle=Predict.at<float>(1)*(4*time_delay+tvec.z/20);
            if_shoot=aim_predict.shoot_logic(pitYaw.y,Predict.at<float>(1),predict_angle);
            pitYaw.y += predict_angle;
            new_number = pitYaw.y;
            if(badcondition = true){
                pitYaw.y = datadealer(pitYaw.y);
            }
        }else{   
            pitYaw = calPitchAndYaw(tvec.x, tvec.y, tvec.z, tvec.z/45, -50, 170, currPitch, currYaw);
        }
        return AIM_TARGET_FOUND;
    }
    return AIM_TARGET_NOT_FOUND;
}

void AutoAim::datajudger(){
    if(new_number != old_number){
        numbase.push_back(new_number);
        if(numbase.size()>=20){
            int i=0;
            for (list<float>::iterator it = numbase.begin(); it != numbase.end(); ++it){
                fft.x[i].real = *it;
                fft.x[i].img = 0;
                i++;
            }
            fft.initW(numbase.size());
            fft.fftx();
            for (i = 0; i < numbase.size(); i++)
            {
                //ofstream outfile("/home/weepies/output2.txt", ios::ate);
                result[i] = sqrt(fft.x[i].real*fft.x[i].real + fft.x[i].img*fft.x[i].img);
                //cout << setprecision(2) << result[i]/numbase.size()*2 << " ************************";
                //outfile<< result[i]/numbase.size()*2<<endl;
                if(i > numbase.size()/2){
                    if(result[i] > frequency)
                        badnum++;
                }
            }
            numbase.pop_front();
        }
        if(badnum>5){
            badcondition = true;
        }
        else{
            badcondition =  false;
        }
        badnum = 0;
        old_number = new_number;
    }
}   

float AutoAim::datadealer(float new_num){
    if (new_num - old_num > 0 )       
        new_flags = 1;
    else new_flags = 0;
    if (new_flags == old_flags)          
    {
        if (abs (new_num - old_num) > Threshold_min)    
            num += 5;
        if (num >= Threshold_max)      
            k += 0.2; 
    }
    else                
    {
        num = 0;      
        k = 0.2;
        old_flags = new_flags; 
    } 
    if (k > 0.95)  {
        k = 0.95;    
        // num_x = 0;
        // k_x = 0.2;
    }
    new_num = (1-k) * old_num + k * new_num;   
    old_num = new_num;      
    return old_num;
}








