#ifndef __kimm_traj_python_Path_hpp__
#define __kimm_traj_python_Path_hpp__

#include "kimm_trajectory_smoother/bindings/fwd.hpp"

#include "kimm_trajectory_smoother/Path.h"

namespace kimmtraj
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename Path>
    struct PathPythonVisitor
    : public boost::python::def_visitor< PathPythonVisitor<Path> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<stdlist_Eigenvec, double> ((bp::arg("waypoint"), bp::arg("maxDeviation")), "Default Constructor"))
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Path info.";
        bp::class_<Path>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(PathPythonVisitor<Path>());
      }
    };

    template<typename Path>
    struct StdlistPythonVisitor
    : public boost::python::def_visitor< StdlistPythonVisitor<Path> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<> ("Default Constructor"))
        .def("extend", &Path::extend, (bp::arg("VectorXd")), "Extend waypoint" )
        .def("clear", &Path::clear, ("Reset Waypoints"))
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Path info.";
        bp::class_<Path>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(StdlistPythonVisitor<Path>());
      }
    };
  }

  
}


#endif // ifndef __kimm_traj_python_Path_hpp__