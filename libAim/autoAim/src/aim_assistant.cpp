#include "aim_assistant.h"
using namespace std;
using namespace cv;
Aim_assistant::Aim_assistant(){}
Aim_assistant::~Aim_assistant(){}
void SSR(Mat frame, Mat frame_ssr, Size size_ssr);
void MSR(Mat frame, Mat *frame_ssr,Mat frame_msr, int size_num);
void SSRCR(Mat frame, Mat frame_ssr, Size size_ssr);
void MSRCR(Mat frame, Mat frame_msrcr, Size size_min, Size size_max, int size_num);
//转换opencv图片至tensorflow格式
void  cvmat_to_tensor(cv::Mat  img,Tensor* tensor,int rows,int cols){
    if(!img.empty()){
       // MSRCR(img, img, Size(21,21), Size(61,61),2);
        //SSR(img,img,Size(5,5));
    cv::resize(img,img,cv::Size(rows,cols));
    //MSRCR(img, img, Size(21,21), Size(61,61),3);
    
    for(int i = 0;i<img.rows;i++){
        for(int j = 0;j<img.cols;j++){
            for(int k = 0; k<3;k++){
                int tmp = (uchar)img.at<Vec3b>(i,j)[k]*1.2+10;
                if(tmp>255) img.at<Vec3b>(i,j)[k] = 2*255 - tmp;
                else img.at<Vec3b>(i,j)[k] = tmp;
            }
        }
    }
    //imshow("k",img);
    cvtColor(img,img,COLOR_BGR2GRAY);
    float *p=tensor->flat<float>().data();
    cv::Mat imagePixels(rows,cols,CV_32FC1,p);
    img.convertTo(imagePixels,CV_32FC1);
    }
}

//初始化过程
int Aim_assistant::init(string model_path){
    Status status = NewSession(SessionOptions(), &session);
    if (!status.ok()) {
        std::cerr << status.ToString() << std::endl;
        return -1;
    } else {
        std::cout << "Session created successfully" << std::endl;
    }
    status = ReadBinaryProto(Env::Default(), model_path, &graph_def);
    if (!status.ok()) {
        std::cerr << status.ToString() << std::endl;
        return -1;
    } else {
        std::cout << "Load graph protobuf successfully" << std::endl;
    }
    status = session->Create(graph_def);
    if (!status.ok()) {
        std::cerr << status.ToString() << std::endl;
        return -1;
    } else {
        std::cout << "Add graph to session successfully" << std::endl;
    }
    Mat black=Mat(Size(32,32),CV_8UC1,Scalar(0));
    cout<<black.size()<<endl;
    
    TensorInit(session,black);
    return 1;


} 
void Aim_assistant::TensorInit(Session* session,Mat& img){
    Tensor x(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,32, 32,1}));
    std::vector<std::pair<string, tensorflow::Tensor>> inputs;
    inputs.push_back(std::pair<std::string, tensorflow::Tensor>("test-input/input", x));
    Tensor tensor_out(tensorflow::DT_FLOAT, TensorShape({1,6}));
    std::vector<tensorflow::Tensor> outputs={{ tensor_out }};
    Status status= session->Run(inputs, {"softmax"}, {}, &outputs);
    if (!status.ok()) {
        cout<<"failure"<<endl;
        std::cout << status.ToString() << "\n";
    }else cout<<"ok"<<endl;

}
int Aim_assistant::check_armor(cv::Mat frame){
    Tensor x(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,32, 32,1}));
    cvmat_to_tensor(frame,&x,32,32);
    std::vector<std::pair<string, tensorflow::Tensor>> inputs;
    inputs.push_back(std::pair<std::string, tensorflow::Tensor>("test-input/input", x));//tensor输入
    Tensor tensor_out(tensorflow::DT_FLOAT, TensorShape({1,6}));
    std::vector<tensorflow::Tensor> outputs={{ tensor_out }};
    Status status= session->Run(inputs, {"softmax"}, {}, &outputs);
    if (!status.ok()) {
        cout<<"failure"<<endl;
        std::cout << status.ToString() << "\n";
        return -1;
    }else{
        Tensor t = outputs[0];
        auto tmap = t.tensor<float, 2>();
        int m=0;
        for(int i=0;i<6;++i){
            cout<<i<<"-->"<<tmap(0,i)<<" ";
            if(tmap(0,i)>=tmap(0,m))
             m=i;
        }
        if(tmap(0,m)>0.6){
            cout<<"result: "<<m+1<<endl;
            return m+1;
        }else return -1;
    }
}