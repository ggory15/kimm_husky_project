#ifndef __solver_hqp_solver_util_hpp__
#define __solver_hqp_solver_util_hpp__

#include "kimm_hqp_controller/solver/fwd.hpp"
#include "kimm_hqp_controller/math/constraint_base.hpp"
#include <string>

#include <iostream>

namespace kimmhqp{
    namespace solver{
    //     std::string HQPDataToString(const HQPData & data, bool printMatrices=false){
    //         using namespace std;
      
    //     stringstream ss;
    //     unsigned int priority = 0;
    //     for(HQPData::const_iterator it=data.begin(); it!=data.end(); it++)
    //     {
    //         ss<<"Level "<< priority<<endl;
    //         for(ConstraintLevel::const_iterator iit=it->begin(); iit!=it->end(); iit++)
    //         {
    //         auto c = iit->second;
    //         ss<<" - "<<c->name()<<": w="<<iit->first<<", ";
    //         if(c->isEquality())
    //             ss<<"equality, ";
    //         else if(c->isInequality())
    //             ss<<"inequality, ";
    //         else
    //             ss<<"bound, ";
    //         ss<<c->rows()<<"x"<<c->cols()<<endl;
    //         }
    //         priority++;
    //     }
        
    //     if(printMatrices)
    //     {
    //         ss<<endl;
    //         for(HQPData::const_iterator it=data.begin(); it!=data.end(); it++)
    //         {
    //         for(ConstraintLevel::const_iterator iit=it->begin(); iit!=it->end(); iit++)
    //         {
    //             auto c = iit->second;
    //             ss<<"*** "<<c->name()<<" *** ";
    //             if(c->isEquality())
    //             {
    //             ss<<"(equality)"<<endl;
    //             ss<<"A =\n"<<c->matrix()<<endl;
    //             ss<<"b = "<<c->vector().transpose()<<endl;
    //             }
    //             else if(c->isInequality())
    //             {
    //             ss<<"(inequality)"<<endl;
    //             ss<<"A =\n"<<c->matrix()<<endl;
    //             ss<<"lb = "<<c->lowerBound().transpose()<<endl;
    //             ss<<"ub = "<<c->upperBound().transpose()<<endl;
    //             }
    //             else
    //             {
    //             ss<<"(bounds)"<<endl;
    //             ss<<"lb = "<<c->lowerBound().transpose()<<endl;
    //             ss<<"ub = "<<c->upperBound().transpose()<<endl;
    //             }
    //             ss<<endl;
    //         }
    //         }
    //     }
    //     return ss.str();
    //     }
    }
}
    
#endif