#include "function_thread.h"
#include <iostream>

FunctionThread::FunctionThread(){}

FunctionThread::~FunctionThread(){}

void FunctionThread::init(std::function<void()> function_){
    function = std::move(function_);
}

void FunctionThread::run(){
    function();
}