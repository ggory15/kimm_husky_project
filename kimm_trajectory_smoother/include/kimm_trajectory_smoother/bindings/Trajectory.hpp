#ifndef __kimm_traj_python_Trajectory_hpp__
#define __kimm_traj_python_Trajectory_hpp__

#include "kimm_trajectory_smoother/bindings/fwd.hpp"

#include "kimm_trajectory_smoother/Trajectory.h"
#include "kimm_trajectory_smoother/Path.h"

namespace kimmtraj
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename Traj>
    struct TrajectoryPythonVisitor
    : public boost::python::def_visitor< TrajectoryPythonVisitor<Traj> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<Path , Eigen::VectorXd , Eigen::VectorXd , double> ((bp::arg("path"), bp::arg("maxVelocity"), bp::arg("maxAcceleration"), bp::arg("timeStep")), "Default Constructor"))
        .def("outputPhasePlaneTrajectory", &Traj::outputPhasePlaneTrajectory, "Generate Trajectory")
        .def("isValid", &Traj::isValid, "is Valid Traj?")
        .def("getDuration", &Traj::getDuration, "get duration of output traj")
        .def("getPosition", &Traj::getPosition, bp::arg("time"))
        .def("getVelocity", &Traj::getVelocity, bp::arg("time"))
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Traj info.";
        bp::class_<Traj>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajectoryPythonVisitor<Traj>());
      }
    };
  }
}


#endif // ifndef __kimm_traj_python_Trajectory_hpp__