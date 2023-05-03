#include <pybind11/pybind11.h>
#include "include/pymultiastar.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
    py::options options;
    options.disable_function_signatures();
    m.doc() = R"mydelimiter(
        PyMultiAStar compiled C++ Module
    )mydelimiter";

    py::class_<PyMultiAStar>(m, "PyMultiAStar")
        .def(py::init<pybind11::array_t<float>, bool, float, float, float, float, float, float, bool>(),
             py::arg("map"), py::arg("allow_diag") = true, py::arg("map_res") = RES, py::arg("obstacle_value") = LARGE_NUMBER,
             py::arg("normalizing_path_cost") = NORM_PATH_COST, py::arg("goal_weight") = GOAL_WEIGHT, 
             py::arg("path_weight") = PATH_WEIGHT, py::arg("keep_nodes") = KEEP_NODES, py::arg("path_w0") = W0)
            //   R"myd(
            //     Creates the multi-goal planner to be used

            //     Args:
            //         map (ArrayFloatMxNxK): 3D NumPy array, often called the voxel grid. 
            //                             index (i,j,k) corresponds to (y, x, z)
            //         allow_diag (bool, optional): Allows diagonal travel in map. Defaults to False.
            //         map_res (float, optional): The length (m) of an edge in the voxel grid. Defaults to 2.0.
            //         obstacle_value (float, optional): The value of an obstacle in the map. Defaults to 1.0.
            //         normalizing_path_cost (float, optional): The length and penalities of a path must
            //                                                 be normalized. This should generally be the
            //                                                 the longest path acceptable. Defaults to 1.0.
            //         goal_weight (float, optional): The weighting of the goal risk. Defaults to 0.5.
            //         path_weight (float, optional): The weighting of the path risk. Defaults to 0.5.
            //         keep_nodes (bool, optional): An optimization parameter which, if set to True,
            //                                     may result in less dynamic memory allocations if many
            //                                     goals are being searched for. Defaults to False.
            //         path_w0 (float, optional): The path cost penalty multiplier when encountering
            //                                     a potential field. Defaults to 1.0.
            //  )myd")
        .def("search_multiple", &PyMultiAStar::search_multiple, py::arg("start_cell"), py::arg("goal_cells"))
        .def("search_single", &PyMultiAStar::search_single_public, py::arg("start_cell"), py::arg("goal_cell"))
        .def_property("normalizing_path_cost", &PyMultiAStar::get_normalizing_path_cost, &PyMultiAStar::set_normalizing_path_cost,
                      "Gets and Sets the normalizing_path_cost used during search");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}

