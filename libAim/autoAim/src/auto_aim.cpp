#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>

#define CELECT_DATA
#define DEBUG_MODE
AutoAim::AutoAim(){}


AutoAim::~AutoAim(){}

void AutoAim::init(Aim_assistant* checker){
    id_checker = checker;
    IMG_WIDTH = 1280;
    IMG_HEIGHT = 720;
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

Point2d cal_x_y_sp(RotatedRect &rect, int is_up){
    float angle = (90-rect.angle)*CV_PI/180;
    Point2d point;
    if(is_up){
        point.x = rect.center.x + rect.size.height/4*cos(angle);
        point.y = rect.center.y - rect.size.height/4*sin(angle);
    } else {
        point.x = rect.center.x - rect.size.height/4*cos(angle);
        point.y = rect.center.y + rect.size.height/4*sin(angle);
    }
    return point;
}

void AutoAim::resetROI(){
    rectROI.x = 0;
    rectROI.y = 0;
    rectROI.width = 1280;
    rectROI.height = 720;
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
    
    //msrcr.MultiScaleRetinexCR(image, image, weight, sigema, 128, 128);
    Mat channel[3], Mask, diff;
    int thresh =100 , substract_thresh = 80;
    resetROI();
    mask = img(rectROI);
    split(mask, channel);
    Mask = channel[1];
    mask = channel[0] - channel[2];
    //medianBlur(img,img,1);
    threshold(Mask, Mask, thresh, 255, THRESH_BINARY);
    threshold(mask, mask, substract_thresh, 255, THRESH_BINARY);
    //imshow("img1",Mask);
    
    Mat element = getStructuringElement( MORPH_RECT, Size(1, 5));
    Mat element1 = getStructuringElement( MORPH_RECT, Size(9, 9));
    dilate( mask, mask, element,Point(-1,-1));
    dilate( Mask, Mask, element1,Point(-1,-1),1);
    blur(Mask,Mask,Size(2,2));
    //threshold(Mask,Mask,thresh,255,THRESH_BINARY);
    bitwise_and(Mask, mask, mask);
    //blur(mask,mask,Size(2,2));
    //threshold(mask,mask,substract_thresh,255,THRESH_BINARY);
    #ifdef DEBUG_MODE
    imshow("img2",mask);
    imshow("img",image);
    waitKey(1);
    #endif
    return true;
}

//寻找灯管
void AutoAim::findLamp_rect(vector<RotatedRect> &pre_armor_lamps){
    pre_armor_lamps.clear();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    findContours(mask, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
    RotatedRect temp;
    float lastCenterX = 0, lastCenterY = 0;
    cout<<"counter size :"<<contours.size()<<endl;
    if(LIKELY(contours.size()<200)){
        for(int i=0;i<contours.size();i++){
            if(contours[i].size()>5){
                temp = adjustRRect(fitEllipseDirect(contours[i]));
                //ellipse(image,temp,Scalar(255,2,255),1);
                if(temp.angle >= 90) temp.angle = temp.angle -180;
                if(abs(temp.angle)>45) continue;//旋转矩形角度小于45度，则忽略
                pre_armor_lamps.push_back(temp);
            }
        }
    }
    //
}
void AutoAim::match_lamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps){
    special_condition = false;
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
    params_max_height_ratio= 2;
    params_max_dis_height_ratio= 6;
    params_max_yx_diff_ratio= 1; //一对灯条最大侧向旋转角
    params_max_allow_angle=3; //一对灯条最大允许角度差
    int size = pre_armor_lamps.size();
    vector<float> diff(size,0x3f3f3f3f);
    vector<float> best_match_index(size,-1);
    //计算灯管匹配之间的花费
    float height_ratio,dist, avg_height, diff_angle, diff_height, ratio, totalDiff,inside_angle,diff_width,dis_height_ratio;
    int i,j;
    if(size == 2){
        if(pre_armor_lamps[0].center.x > pre_armor_lamps[1].center.x){
            swap(pre_armor_lamps[0],pre_armor_lamps[1]);//确保偶数为左灯条，奇数为右灯条
        }
        if(pre_armor_lamps[0].angle > 3 && pre_armor_lamps[0].angle < 15 && pre_armor_lamps[1].angle < -3 && pre_armor_lamps[1].angle > -15){
                special_condition = true;
                cout<<"in special mode !!!!!!!!!!!"<<endl;
                real_armor_lamps.push_back(pre_armor_lamps[0]);
                real_armor_lamps.push_back(pre_armor_lamps[1]);
        }
    }
    if(LIKELY(!special_condition)){
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
}
void AutoAim::selectArmorH(vector<RotatedRect> real_armor_lamps){
    float pre=1000;//预定义
    int target_index=-1;//目标索引
    bestCenter.x=-1;
    for(int i=0; i<real_armor_lamps.size(); i+=2){
        if(i+1 >= real_armor_lamps.size()) break;
        Rect armor_area;
        if(real_armor_lamps[i].center.x > real_armor_lamps[i+1].center.x){
            swap(real_armor_lamps[i],real_armor_lamps[i+1]);//确保偶数为左灯条，奇数为右灯条
        }
        //优先选择趋近于图像中心的装甲板,达到操作手指示优先级的功能
        float y = fabs(360 - (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2);
        float x = fabs(640-(real_armor_lamps[i].center.x + real_armor_lamps[i+1].center.x)/2);
        y = y / 360;
        x = x / 640;
        float score = 0.5 * y + 0.5 * x;
        if(score < pre){
            pre = score;
            target_index = i;
        }
    }
    if(target_index != -1){
        bestCenter.x = (real_armor_lamps[target_index].center.x + real_armor_lamps[target_index+1].center.x)/2 + rectROI.x;
        bestCenter.y = (real_armor_lamps[target_index].center.y + real_armor_lamps[target_index+1].center.y)/2 + rectROI.y;
    }
    if(bestCenter.x!=-1){
        clock_t finish = clock();
        best_lamps[0] = real_armor_lamps[target_index];
        best_lamps[1] = real_armor_lamps[target_index+1];
        best_lamps[0].center.x+=rectROI.x;
        best_lamps[0].center.y+=rectROI.y;
        best_lamps[1].center.x+=rectROI.x;
        best_lamps[1].center.y+=rectROI.y;
        rectROI.x = (best_lamps[0].center.x + best_lamps[1].center.x)/2 - (best_lamps[1].center.x - best_lamps[0].center.x);
        rectROI.y = (best_lamps[0].center.y + best_lamps[1].center.y)/2 - (best_lamps[0].size.height + best_lamps[1].size.height)/2;
        rectROI.height = best_lamps[0].size.height + best_lamps[1].size.height;
        rectROI.width = 2*(best_lamps[1].center.x - best_lamps[0].center.x);
        if(!makeRectSafe(rectROI)){
            resetROI();
	    }
    }

}
void AutoAim::select_armor(vector<RotatedRect> real_armor_lamps){
    int lowerY=1000;
    int lowerIndex=-1;
    int hero_index=-1;
    bestCenter.x=-1;
    vector<int> armor_detected;
    if(special_condition){
        pnpSolver.clearPoints3D();
        pnpSolver.pushPoints3D(0, -35, 0);
        pnpSolver.pushPoints3D(0,  -17.5, 0);
        pnpSolver.pushPoints3D(0, 17.5, 0);
        pnpSolver.pushPoints3D(0, 35, 0);
    }
    else{
        //最优装甲板逻辑
        for(int i=0; i<real_armor_lamps.size(); i+=2){
            if(i+1 >= real_armor_lamps.size()) break;
            Rect armor_area;
            if(real_armor_lamps[i].center.x > real_armor_lamps[i+1].center.x){
                swap(real_armor_lamps[i],real_armor_lamps[i+1]);//确保偶数为左灯条，奇数为右灯条
            }
            //计算装甲板灯条四个点像素坐标
            Point2d left_up = cal_x_y(real_armor_lamps[i],1);
            Point2d left_d = cal_x_y(real_armor_lamps[i],0);
            Point2d right_up = cal_x_y(real_armor_lamps[i+1],1);
            Point2d right_d = cal_x_y(real_armor_lamps[i+1],0);

            //选定装甲板中心区域
            armor_area.x = left_up.x;
            int y = (left_up.y + right_up.y) / 2 - 0.4 * real_armor_lamps[i].size.height;
            if(y<0) armor_area.y;
            else armor_area.y = y;
            if(left_d.y>right_d.y){
                if((1.6) * (left_d.y - right_up.y)+armor_area.y<720)
                    armor_area.height = (1.6) * (left_d.y - right_up.y);
                else{
                    armor_area.height = 720 - armor_area.y;
                }
            }else{
                if((1.6) * (right_d.y - left_up.y)+armor_area.y<720)
                    armor_area.height = (1.6) * (right_d.y - left_up.y);
                else{
                    armor_area.height = 720 - armor_area.y;
                }
            }    
            
            armor_area.width = abs(right_up.x - left_up.x);
            clock_t begin, finish;
            begin = clock() ;
            int number = -1;
            if(armor_area.width != 0 && armor_area.height != 0 ){
                number = id_checker->check_armor(image(armor_area)); 
                #ifdef CELECT_DATA
                Mat img_a;
                cv::resize(image(armor_area),img_a,cv::Size(32,32));
                for(int i = 0;i<img_a.rows;i++){
                    for(int j = 0;j<img_a.cols;j++){
                        for(int k = 0; k<3;k++){
                            int tmp = (uchar)img_a.at<Vec3b>(i,j)[k]*1.2+10;
                            if(tmp>255) img_a.at<Vec3b>(i,j)[k] = 2*255 - tmp;
                            else img_a.at<Vec3b>(i,j)[k] = tmp;
                        }
                    }
                }
                cvtColor(img_a,img_a,COLOR_BGR2GRAY); //输入CNN模型预测 0-4 对应 1-5 5种装甲板
                imwrite("../armor_data/3/"+to_string(c)+"x1000.png",img_a);
                #endif
            }
        
            c++;
            finish = clock();
            cout<<"CNN time cost: "<<(double)(finish - begin) / CLOCKS_PER_SEC<<endl;
            armor_detected.push_back(number);
            cout<<"number: "<<i<<"   "<<number<<endl;
        }
        // imshow("image",image);
        // waitKey(1);
        for(int i=0;i<armor_detected.size();i++){
            if(armor_detected[i] == 2){
                pnpSolver.clearPoints3D();
                pnpSolver.pushPoints3D(-112, -35, 0);
                pnpSolver.pushPoints3D(112,  -35, 0);
                pnpSolver.pushPoints3D(112, 35, 0);
                pnpSolver.pushPoints3D(-112, 35, 0);
	            cout<<"find hero !!!!!!"<<endl;
                hero_index = 2 * i;
                lowerIndex = 2 * i;
                break;
            }

            //不瞄准工程车
            else if(armor_detected[i] != 5 && armor_detected[i] != -1 && armor_detected[i]!=-1){
                cout<<"found: "<<armor_detected[i]<<" armor "<<endl;
                float y = 720 - (real_armor_lamps[2*i].center.y + real_armor_lamps[2*i+1].center.y)/2;
                float x = fabs(640-(real_armor_lamps[2*i].center.x + real_armor_lamps[2*i+1].center.x)/2);
                y = y / 720;
                x = x / 640;
                float score = 0.6 * y + 0.4 * x;
                if(score < lowerY){
                    lowerY = score;
                    lowerIndex = 2 * i;
                }
            }
        }
    }
    if(special_condition){
        bestCenter.x = real_armor_lamps[0].center.x  + rectROI.x;
        bestCenter.y = real_armor_lamps[0].center.y + rectROI.y;   
    }
    if(hero_index!=-1){
        resizeCount=0;
        count++;
        int height = (real_armor_lamps[hero_index].size.height + real_armor_lamps[hero_index+1].size.height)/2;
        //当灯条高度小于5个像素点时放弃锁定，重新寻找合适目标
        if(LIKELY(height > 1)){
            
            bestCenter.x = (real_armor_lamps[hero_index].center.x 
                        + real_armor_lamps[hero_index+1].center.x)/2 + rectROI.x ;
            bestCenter.y = (real_armor_lamps[hero_index].center.y 
                        + real_armor_lamps[hero_index+1].center.y)/2 +rectROI.y;
        }else{
            resetROI();
            count=0;
        }
    }
    //优先锁定图像下方且靠近中心装甲板
    else if(lowerIndex == -1){
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
    }

    if(bestCenter.x!=-1){
        if(special_condition){
            best_lamps[0] = real_armor_lamps[0];
            best_lamps[1] = real_armor_lamps[1];
            best_lamps[0].center.x+=rectROI.x;
            best_lamps[0].center.y+=rectROI.y;
            best_lamps[1].center.x+=rectROI.x;
            best_lamps[1].center.y+=rectROI.y;
            resetROI();
        }else{
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
}
BaseAim::AimResult AutoAim::aim(Mat &src, float currPitch, float currYaw, Point2f &pitYaw,int is_predict,bool &if_shoot,float time_delay){

    bool isKalman = true;
    
    if(bestCenter.x!=-1){
        if(!special_condition){
            circle(src, bestCenter, 20, Scalar(255,255,255), 5);
            count++;
            Mat frame;
            src.copyTo(frame);
            Rect rect;
            Point2d left_up = cal_x_y(best_lamps[0],1);
            Point2d right_up = cal_x_y(best_lamps[1],1);
            pnpSolver.pushPoints2D(left_up);//P1
            pnpSolver.pushPoints2D(right_up);//P3
            pnpSolver.pushPoints2D(cal_x_y(best_lamps[1],0));//P2
            pnpSolver.pushPoints2D(cal_x_y(best_lamps[0],0));//P4
            pnpSolver.solvePnP();
            pnpSolver.clearPoints2D();
            //pnpSolver.showParams();

            Point3d tvec = pnpSolver.getTvec();
            if(is_predict && isKalman){
	            pitYaw = calPitchAndYaw(tvec.x,tvec.y, tvec.z, tvec.z/63, -120, 170, currPitch, currYaw);
	            measurement.at<float>(0) = currYaw + pitYaw.y;   

                if(count==1){
                    Mat statePost=(Mat_<float>(2, 1) << currYaw+pitYaw.y,0);
                    aim_predict.model_init();
                    aim_predict.reset_kf_statepost(statePost);
                } 
                Mat Predict = this->aim_predict.predict(measurement,time_delay);
                float predict_angle=Predict.at<float>(1)*(6*time_delay);
                if_shoot=aim_predict.shoot_logic(pitYaw.y,Predict.at<float>(1),predict_angle);
                pitYaw.y += predict_angle;
                new_number = pitYaw.y;
                if(0){
                    pitYaw.y = datadealer(pitYaw.y);
                }
            }else{   
                pitYaw = calPitchAndYaw(tvec.x, tvec.y, tvec.z, tvec.z/45, -50, 170, currPitch, currYaw);
            }
        }else{
            Point2d up = cal_x_y(best_lamps[0],1);
            Point2d down = cal_x_y(best_lamps[0],0);
            pnpSolver.pushPoints2D(up);//P1
            pnpSolver.pushPoints2D(cal_x_y_sp(best_lamps[0],1));//P2
            pnpSolver.pushPoints2D(cal_x_y_sp(best_lamps[0],0));//P4
            pnpSolver.pushPoints2D(down);//P3
            pnpSolver.solvePnP();
            pnpSolver.clearPoints2D();
            Point3d tvec = pnpSolver.getTvec();
            pitYaw = calPitchAndYaw(tvec.x,tvec.y, tvec.z, tvec.z/63, -120, 170, currPitch, currYaw);
            pitYaw.y -= 1;
        }
        return AIM_TARGET_FOUND;
    }
    count = 0;
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








