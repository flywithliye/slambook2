#include<iostream>
  
int main(){
 
#if defined  __aarch64__
    std::cout<<"this is arm cpu"<<std::endl;
#elif defined __x86_64__
    std::cout<<"this id x86 cpu"<<std::endl;
#endif
    return 0;
}
