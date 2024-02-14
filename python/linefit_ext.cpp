/*!
 * linefit nb module file
 *
 * @author Qingwen Zhang (https://kin-zhang.github.io/)
 * @version 1.0.0
 * @date 2024-02-14 20:13
 *
 * @copyright Copyright (c) 2022, Qingwen Zhang, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Qingwen Zhang, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Eigen/Core>
#include "nanobind/nanobind.h"
// #include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/ndarray.h>
// #include <nanobind/stl/bind_vector.h>

#include "../cpp/linefit/ground_segmentation.h"
#include "../cpp/linefit/mics.h"
// #include "stl_vector_eigen.h"

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(linefit, m) {
    nb::class_<GroundSegmentation>(m, "ground_seg")
        .def(nb::init<>(), "linefit ground segmentation constructor, param: TODO")
        // .def("run", nb::overload_cast<std::vector<Eigen::Vector3d> &>(&GroundSegmentation::segment), "points"_a);
        .def("run", &GroundSegmentation::segment, "points"_a, nanobind::rv_policy::reference);
}