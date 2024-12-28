#include"mathlibrary.h"

Math::Math()
{

}

float Math::add(float a, float b){
    return a+b;
}

float Math::sub(float a, float b){
    return a-b;
}

float Math::multiply(float a, float b){
    return a*b;
}

float Math::div(float a, float b){
    if(b==0) return NAN;
    return a/b;
}



