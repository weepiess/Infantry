#include "datathread.h"
#include <iostream>

Thread::Thread(){}

Thread::~Thread(){}

void Thread::init(AutoAim *pAutoaim){
    autoaim = pAutoaim;
}

void Thread::run(){
    while(true)
        autoaim -> datajudger();
}