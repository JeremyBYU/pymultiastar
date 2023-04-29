#include <pybind11/pybind11.h>
#include "include/pymultiastar.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
    m.doc() = R"pbdoc(
        PyMultiAstar
        -----------------------

        .. currentmodule:: pymultiastar

        .. autosummary::
           :toctree: _generate

    )pbdoc";

    py::class_<PyMultiAStar>(m, "PyMultiAStar")
        .def(py::init<pybind11::array_t<float>, bool, float, float, float, float, float, float, bool>(),
             py::arg("map"), py::arg("allow_diag") = true, py::arg("map_res") = RES, py::arg("obstacle_value") = LARGE_NUMBER,
             py::arg("normalizing_path_cost") = NORM_PATH_COST, py::arg("goal_weight") = GOAL_WEIGHT, py::arg("path_weight") = PATH_WEIGHT, py::arg("keep_nodes") = KEEP_NODES, py::arg("path_w0") = W0)
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

