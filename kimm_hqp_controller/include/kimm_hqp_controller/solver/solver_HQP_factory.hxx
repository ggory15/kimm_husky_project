#ifndef __solver_hqp_factory_hxx__
#define __solver_hqp_factory_hxx__

#include "kimm_hqp_controller/solver/solver_HQP_factory.hpp"
#include "kimm_hqp_controller/solver/solver_HQP_eiquadprog_rt.hxx"

namespace kimmhqp{
    namespace solver{
        template<int nVars, int nEqCon, int nIneqCon>
        SolverHQPBase* SolverHQPFactory::createNewSolver(const SolverHQP solverType,
                                                        const std::string & name)
        {
        if(solverType==SOLVER_HQP_EIQUADPROG_RT)
            return new SolverHQuadProgRT<nVars, nEqCon, nIneqCon>(name);
        
        assert(false && "Specified solver type not recognized");
        return NULL;
        }
    }
}

#endif